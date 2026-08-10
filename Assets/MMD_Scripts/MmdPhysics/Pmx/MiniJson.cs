// ===========================================================================
// 依存ゼロの最小 JSON パーサ (GLB の glTF JSON / extras.mmd 読み取り用)。
// Unity 標準 JsonUtility は入れ子・動的配列・任意キーに弱く、System.Text.Json や
// Newtonsoft は Unity 既定に含まれず依存が増えるため、自作の再帰下降パーサを用いる。
// 出力: object = Dictionary<string,object> / List<object> / string / double / bool / null。
// C# 9 / .NET Standard 2.1 (Unity 6) で動作。読み取り専用・本体物理には非関与。
// ===========================================================================
using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace BulletPhysics.Pmx
{
    public static class MiniJson
    {
        public static object Parse(string s)
        {
            int i = 0;
            var v = ParseValue(s, ref i);
            SkipWs(s, ref i);
            return v;
        }

        // --- アクセス補助 ---
        public static Dictionary<string, object> Obj(object o) => o as Dictionary<string, object>;
        public static List<object> Arr(object o) => o as List<object>;
        public static string Str(object o) => o as string;
        public static double Num(object o) => o is double d ? d : (o is bool b ? (b ? 1 : 0) : 0);
        public static int Int(object o) => (int)Math.Round(Num(o));
        public static float Flt(object o) => (float)Num(o);

        public static object Get(Dictionary<string, object> o, string key)
            => (o != null && o.TryGetValue(key, out var v)) ? v : null;

        // --- パース本体 ---
        private static object ParseValue(string s, ref int i)
        {
            SkipWs(s, ref i);
            char c = s[i];
            switch (c)
            {
                case '{': return ParseObject(s, ref i);
                case '[': return ParseArray(s, ref i);
                case '"': return ParseString(s, ref i);
                case 't': i += 4; return true;   // true
                case 'f': i += 5; return false;  // false
                case 'n': i += 4; return null;   // null
                default: return ParseNumber(s, ref i);
            }
        }

        private static Dictionary<string, object> ParseObject(string s, ref int i)
        {
            var o = new Dictionary<string, object>();
            i++; // {
            SkipWs(s, ref i);
            if (s[i] == '}') { i++; return o; }
            while (true)
            {
                SkipWs(s, ref i);
                string key = ParseString(s, ref i);
                SkipWs(s, ref i);
                i++; // :
                o[key] = ParseValue(s, ref i);
                SkipWs(s, ref i);
                char c = s[i++];
                if (c == '}') break;
                // c == ','
            }
            return o;
        }

        private static List<object> ParseArray(string s, ref int i)
        {
            var a = new List<object>();
            i++; // [
            SkipWs(s, ref i);
            if (s[i] == ']') { i++; return a; }
            while (true)
            {
                a.Add(ParseValue(s, ref i));
                SkipWs(s, ref i);
                char c = s[i++];
                if (c == ']') break;
                // c == ','
            }
            return a;
        }

        private static string ParseString(string s, ref int i)
        {
            var sb = new StringBuilder();
            i++; // opening quote
            while (true)
            {
                char c = s[i++];
                if (c == '"') break;
                if (c == '\\')
                {
                    char e = s[i++];
                    switch (e)
                    {
                        case '"': sb.Append('"'); break;
                        case '\\': sb.Append('\\'); break;
                        case '/': sb.Append('/'); break;
                        case 'b': sb.Append('\b'); break;
                        case 'f': sb.Append('\f'); break;
                        case 'n': sb.Append('\n'); break;
                        case 'r': sb.Append('\r'); break;
                        case 't': sb.Append('\t'); break;
                        case 'u':
                            int code = int.Parse(s.Substring(i, 4), NumberStyles.HexNumber, CultureInfo.InvariantCulture);
                            i += 4;
                            sb.Append((char)code);
                            break;
                        default: sb.Append(e); break;
                    }
                }
                else sb.Append(c);
            }
            return sb.ToString();
        }

        private static object ParseNumber(string s, ref int i)
        {
            int start = i;
            while (i < s.Length)
            {
                char c = s[i];
                if (c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E' || (c >= '0' && c <= '9')) i++;
                else break;
            }
            return double.Parse(s.Substring(start, i - start), NumberStyles.Float, CultureInfo.InvariantCulture);
        }

        private static void SkipWs(string s, ref int i)
        {
            while (i < s.Length)
            {
                char c = s[i];
                if (c == ' ' || c == '\t' || c == '\n' || c == '\r') i++;
                else break;
            }
        }
    }
}
