using UnityEngine;

public class FreeCameraController : MonoBehaviour
{
    [Header("Movement")]
    public float moveSpeed = 5f;
    public float sprintMultiplier = 2f;

    [Header("Rotation")]
    public float mouseSensitivity = 1.5f;
    public float keyRotationSpeed = 90f;
    public float keyRotationSprint = 180f;
    public float maxPitch = 85f;

    private float yaw;
    private float pitch;

    void Update()
    {
        // --- 移動 ---
        float speed = Input.GetKey(KeyCode.LeftShift) ? moveSpeed * sprintMultiplier : moveSpeed;

        float h = Input.GetAxis("Horizontal"); // A D
        float v = Input.GetAxis("Vertical");   // W S

        float upDown = 0f;
        if (Input.GetKey(KeyCode.Space)) upDown += 1f;
        if (Input.GetKey(KeyCode.LeftControl)) upDown -= 1f;

        Vector3 move =
            transform.forward * v +
            transform.right * h +
            transform.up * upDown;

        transform.position += move * speed * Time.deltaTime;

        // --- マウス回転 ---
        yaw += Input.GetAxis("Mouse X") * mouseSensitivity;
        pitch -= Input.GetAxis("Mouse Y") * mouseSensitivity;

        // --- 矢印キー回転 ---
        float rotSpeed = Input.GetKey(KeyCode.LeftShift) ? keyRotationSprint : keyRotationSpeed;

        if (Input.GetKey(KeyCode.LeftArrow)) yaw -= rotSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.RightArrow)) yaw += rotSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.UpArrow)) pitch -= rotSpeed * Time.deltaTime;
        if (Input.GetKey(KeyCode.DownArrow)) pitch += rotSpeed * Time.deltaTime;

        pitch = Mathf.Clamp(pitch, -maxPitch, maxPitch);

        transform.rotation = Quaternion.Euler(pitch, yaw, 0);
    }
}
