using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Expansion : MonoBehaviour
{
    public MeshFilter meshFilter;
    public Mesh mesh;
    public Vector3[] points;
    void Start()
    {
        meshFilter = GetComponent<MeshFilter>();
        mesh = meshFilter.mesh;
        points = mesh.vertices;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
