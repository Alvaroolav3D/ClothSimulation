using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class movemet : MonoBehaviour
{
    //Variables
    public float speed = .01f;
    private Vector3 moveDirection = Vector3.zero;

    void Update()
    {
        moveDirection = new Vector3(Input.GetAxis("Horizontal"), Input.GetAxis("Vertical"), 0);
        moveDirection *= speed;
        transform.position += moveDirection;


    }
}
