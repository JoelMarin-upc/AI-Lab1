using UnityEngine;

[RequireComponent(typeof(CharacterController))]
public class Drive : MonoBehaviour
{
    public float speed = 10.0f;
    public float rotationSpeed = 100.0f;
    public float currentSpeed = 0;
    public float gravity = -9.81f;

    CharacterController controller;
    float verticalVelocity;

    void Start()
    {
        controller = GetComponent<CharacterController>();
    }

    void Update()
    {
        // Arrow keys / WASD, values from -1 to 1
        float translation = Input.GetAxis("Vertical") * speed;
        float rotation = Input.GetAxis("Horizontal") * rotationSpeed;

        // Per second instead of per frame
        translation *= Time.deltaTime;
        rotation *= Time.deltaTime;

        // Rotate around the y-axis
        transform.Rotate(0, rotation, 0);

        // Gravity keeps the cop on the floor
        if (controller.isGrounded && verticalVelocity < 0f)
            verticalVelocity = -2f;
        verticalVelocity += gravity * Time.deltaTime;

        // Move along the forward direction; the controller stops at colliders
        Vector3 move = transform.forward * translation;
        move.y = verticalVelocity * Time.deltaTime;
        controller.Move(move);

        currentSpeed = translation;
    }
}