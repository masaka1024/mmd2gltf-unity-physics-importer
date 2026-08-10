// ===========================================================================
// Unity Bullet 互換物理エンジン – SoftBody (PMX 2.1)
// btSoftBody 相当の質点-バネ系。TriMesh / Rope、B-Link (Bending)、
// アンカー剛体、Pin 頂点に対応。クラスタ/AeroModel は簡易対応。
// 仕様: bullet 2.75 基準 (極北P PMX仕様 2.1)。
// ===========================================================================

using System;
using System.Collections.Generic;

namespace BulletPhysics
{
    /// <summary>SoftBody のノード (質点)。btSoftBody::Node 相当。</summary>
    public struct SoftNode
    {
        public Vec3 X;      // 位置 (m_x)
        public Vec3 Xprev;  // 前ステップ位置 (Verlet 用)
        public Vec3 V;      // 速度 (m_v)
        public Vec3 N;      // 法線 (m_n)
        public float InvMass; // m_im (0 = Pin)
    }

    /// <summary>ノード間リンク (距離拘束)。btSoftBody::Link 相当。</summary>
    public struct SoftLink
    {
        public int A, B;
        public float RestLength;
        public float Stiffness; // m_kLST 相当 [0,1]
    }

    /// <summary>アンカー: SoftBody ノードを剛体へ拘束。</summary>
    public struct SoftAnchor
    {
        public int Node;
        public RigidBody Body;
        public Vec3 LocalPivot; // 剛体ローカルのアンカー位置
        public bool NearMode;
    }

    public enum SoftBodyShape { TriMesh = 0, Rope = 1 }

    /// <summary>
    /// 質点-バネ SoftBody。Position-Based (投影反復) でリンク距離拘束を解く。
    /// </summary>
    public sealed class SoftBody
    {
        public string Name = "";
        public SoftBodyShape Shape;
        public byte Group;
        public ushort CollisionMask; // bit=1 で「そのグループと衝突する」

        public readonly List<SoftNode> Nodes = new();
        public readonly List<SoftLink> Links = new();
        public readonly List<SoftAnchor> Anchors = new();
        public readonly List<int> Faces = new();  // TriMesh: 3 連続で 1 面

        // Config (PMX <config> 抜粋)。
        public float DP = 0.0f;   // Damping [0,1]
        public float DF = 0.2f;   // Dynamic friction [0,1]
        public float LST = 1.0f;  // Linear stiffness [0,1] (Material)
        public float MT = 0.0f;   // Pose matching [0,1] (未使用簡易)
        public float AnchorHardness = 1.0f; // kAHR
        public int PositionIterations = 4;  // P_IT

        public Vec3 Gravity = new(0, -9.8f, 0);

        // --- 構築ヘルパー ---

        /// <summary>直列頂点列から Rope を作成 (btSoftBodyHelpers::CreateRope 改変相当)。</summary>
        public static SoftBody CreateRope(IList<Vec3> points, float totalMass, float stiffness)
        {
            var sb = new SoftBody { Shape = SoftBodyShape.Rope, LST = stiffness };
            float im = points.Count > 0 ? points.Count / Math.Max(1e-6f, totalMass) : 0f;
            foreach (var p in points)
                sb.Nodes.Add(new SoftNode { X = p, Xprev = p, InvMass = im });
            for (int i = 0; i < points.Count - 1; i++)
                sb.AddLink(i, i + 1, stiffness);
            return sb;
        }

        /// <summary>三角メッシュから TriMesh SoftBody を作成。</summary>
        public static SoftBody CreateFromTriMesh(IList<Vec3> verts, IList<int> indices,
            float totalMass, float stiffness)
        {
            var sb = new SoftBody { Shape = SoftBodyShape.TriMesh, LST = stiffness };
            float im = verts.Count > 0 ? verts.Count / Math.Max(1e-6f, totalMass) : 0f;
            foreach (var p in verts)
                sb.Nodes.Add(new SoftNode { X = p, Xprev = p, InvMass = im });

            var edgeSet = new HashSet<long>();
            for (int i = 0; i + 2 < indices.Count; i += 3)
            {
                int a = indices[i], b = indices[i + 1], c = indices[i + 2];
                sb.Faces.Add(a); sb.Faces.Add(b); sb.Faces.Add(c);
                sb.TryAddEdge(edgeSet, a, b, stiffness);
                sb.TryAddEdge(edgeSet, b, c, stiffness);
                sb.TryAddEdge(edgeSet, c, a, stiffness);
            }
            return sb;
        }

        private void TryAddEdge(HashSet<long> set, int a, int b, float stiffness)
        {
            long key = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
            if (set.Add(key)) AddLink(a, b, stiffness);
        }

        public void AddLink(int a, int b, float stiffness)
        {
            var len = (Nodes[b].X - Nodes[a].X).Length;
            Links.Add(new SoftLink { A = a, B = b, RestLength = len, Stiffness = stiffness });
        }

        /// <summary>B-Link (Bending): 距離 d 離れた頂点間にリンクを追加。</summary>
        public void GenerateBendingConstraints(int distance, float stiffness)
        {
            // 簡易: リンクグラフ上で BFS 距離が distance のノード対を接続。
            int n = Nodes.Count;
            var adj = new List<int>[n];
            for (int i = 0; i < n; i++) adj[i] = new List<int>();
            foreach (var l in Links) { adj[l.A].Add(l.B); adj[l.B].Add(l.A); }

            var edgeSet = new HashSet<long>();
            foreach (var l in Links)
            {
                long key = l.A < l.B ? ((long)l.A << 32) | (uint)l.B : ((long)l.B << 32) | (uint)l.A;
                edgeSet.Add(key);
            }

            for (int s = 0; s < n; s++)
            {
                var dist = BfsDepth(adj, s, distance);
                foreach (var (node, d) in dist)
                    if (d == distance && node > s)
                        TryAddEdge(edgeSet, s, node, stiffness);
            }
        }

        private static List<(int, int)> BfsDepth(List<int>[] adj, int start, int maxDepth)
        {
            var visited = new Dictionary<int, int> { { start, 0 } };
            var queue = new Queue<int>();
            queue.Enqueue(start);
            var result = new List<(int, int)>();
            while (queue.Count > 0)
            {
                int cur = queue.Dequeue();
                int d = visited[cur];
                if (d >= maxDepth) continue;
                foreach (var nx in adj[cur])
                {
                    if (visited.ContainsKey(nx)) continue;
                    visited[nx] = d + 1;
                    result.Add((nx, d + 1));
                    queue.Enqueue(nx);
                }
            }
            return result;
        }

        /// <summary>アンカー追加 (btSoftBody::appendAnchor 相当)。</summary>
        public void AppendAnchor(int node, RigidBody body, bool nearMode)
        {
            var pivot = body.WorldTransform.InverseTransformPoint(Nodes[node].X);
            Anchors.Add(new SoftAnchor { Node = node, Body = body, LocalPivot = pivot, NearMode = nearMode });
        }

        /// <summary>Pin 設定: ノードの逆質量を 0 に (m_im = 0)。</summary>
        public void SetPin(int node, bool pinned)
        {
            var nd = Nodes[node];
            nd.InvMass = pinned ? 0f : nd.InvMass;
            Nodes[node] = nd;
        }

        public void SetTotalMass(float mass)
        {
            int movable = 0;
            foreach (var n in Nodes) if (n.InvMass > 0) movable++;
            if (movable == 0) return;
            float im = movable / mass;
            for (int i = 0; i < Nodes.Count; i++)
            {
                var n = Nodes[i];
                if (n.InvMass > 0) { n.InvMass = im; Nodes[i] = n; }
            }
        }

        /// <summary>Pin 頂点の位置を外部 (ボーン変形) から直接設定 (m_x)。</summary>
        public void SetNodePosition(int node, Vec3 pos)
        {
            var n = Nodes[node];
            n.X = pos;
            Nodes[node] = n;
        }

        // --- シミュレーション 1 ステップ (Position-Based Dynamics) ---
        public void Step(float dt)
        {
            // 1. 予測 (semi-implicit + Verlet 速度)。
            for (int i = 0; i < Nodes.Count; i++)
            {
                var n = Nodes[i];
                if (n.InvMass > 0)
                {
                    n.V += Gravity * dt;
                    n.V *= Math.Max(0f, 1f - DP);
                    n.Xprev = n.X;
                    n.X += n.V * dt;
                }
                Nodes[i] = n;
            }

            // 2. リンク距離拘束を反復投影。
            for (int it = 0; it < PositionIterations; it++)
            {
                SolveLinks();
                SolveAnchors();
            }

            // 3. 速度再計算 + 法線更新。
            float invDt = dt > 0 ? 1f / dt : 0f;
            for (int i = 0; i < Nodes.Count; i++)
            {
                var n = Nodes[i];
                if (n.InvMass > 0)
                    n.V = (n.X - n.Xprev) * invDt;
                Nodes[i] = n;
            }
            UpdateNormals();
        }

        private void SolveLinks()
        {
            for (int i = 0; i < Links.Count; i++)
            {
                var l = Links[i];
                var na = Nodes[l.A]; var nb = Nodes[l.B];
                float wSum = na.InvMass + nb.InvMass;
                if (wSum <= 0) continue;

                var delta = nb.X - na.X;
                float len = delta.Length;
                if (len < 1e-8f) continue;
                float diff = (len - l.RestLength) / len;
                var corr = delta * (diff * l.Stiffness);

                na.X += corr * (na.InvMass / wSum);
                nb.X -= corr * (nb.InvMass / wSum);
                Nodes[l.A] = na; Nodes[l.B] = nb;
            }
        }

        private void SolveAnchors()
        {
            foreach (var a in Anchors)
            {
                var n = Nodes[a.Node];
                if (n.InvMass <= 0) continue;
                var target = a.Body.WorldTransform.TransformPoint(a.LocalPivot);
                n.X += (target - n.X) * AnchorHardness;
                Nodes[a.Node] = n;
            }
        }

        private void UpdateNormals()
        {
            for (int i = 0; i < Nodes.Count; i++)
            {
                var n = Nodes[i]; n.N = Vec3.Zero; Nodes[i] = n;
            }
            for (int f = 0; f + 2 < Faces.Count; f += 3)
            {
                int a = Faces[f], b = Faces[f + 1], c = Faces[f + 2];
                var normal = Vec3.Cross(Nodes[b].X - Nodes[a].X, Nodes[c].X - Nodes[a].X);
                Accumulate(a, normal); Accumulate(b, normal); Accumulate(c, normal);
            }
            for (int i = 0; i < Nodes.Count; i++)
            {
                var n = Nodes[i];
                if (n.N.LengthSquared > 1e-12f) n.N = n.N.Normalized;
                Nodes[i] = n;
            }
        }

        private void Accumulate(int i, Vec3 normal)
        {
            var n = Nodes[i]; n.N += normal; Nodes[i] = n;
        }
    }
}
