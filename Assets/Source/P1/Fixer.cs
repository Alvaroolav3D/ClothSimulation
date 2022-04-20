using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Fixer : MonoBehaviour {

    public bool CalculateCollision(Vector3 pos)
    {
        Bounds bounds = GetComponent<Collider>().bounds; //almaceno los limites del collider del objeto
        return bounds.Contains(pos);
    }
}
