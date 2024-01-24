using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class XPBDBox : MonoBehaviour
{
    public float stiffness = 1;
    [SerializeField] private float stiffnessInverse = 1;
    public float constrainDamping;

    public int XPBDIterationNums = 5;

    private float SubDt;
    //mass points
    public Vector3[] PointsPosition;
    private Vector3[] PrevPointsPosition;
    public Vector3[] PBDCorrection;
    private Vector3[] PointsVelocity;
    private Vector3[] PointsAcceleration;
    private float[] PointsMass;

    private float[] PointsInvMass;
    //for volume conservation
    public Vector3[] FaceNoramls;
    private float[] FaceAreas;

    private int[,] FaceConnectionPointIndex;
    private int[,] FaceEdge;
    private int[,] DiagonalEdge;
    public float[] FaceConstrainRestLength;
    public float[] DiagionalConstrainRestLength;
    [SerializeField] private float[] RestTetVolume;

    private float RestVolume;

    //components
    private BoxCollider MCollider;
    private MeshFilter ChildMeshFilter;
    private Mesh ChildMesh;
    private Bounds ChildMeshBounds;
    private Transform ChildTransform;
    private Vector3 InitialTransformPosition;
    private Vector3 InitialTransformScale;
    
    //mapping
    private Vector3 MappingCoef;
    private Vector3 InitialOffset;
    
    //debug
    public float FloorHeight;
    
    void Start()
    {
        //get components
        MCollider = GetComponent<BoxCollider>();
        ChildMeshFilter = GetComponentInChildren<MeshFilter>();

        ChildMesh = ChildMeshFilter.mesh;
        ChildMeshBounds = ChildMesh.bounds;

        ChildTransform = transform.GetChild(0);
        
        //calculate points'initial positions
        Vector3 localBoundsCenter = ChildMeshBounds.center;
        Vector3 worldBoundsCenter = ChildTransform.TransformPoint(localBoundsCenter);
        MCollider.center = transform.InverseTransformPoint(worldBoundsCenter);
        Vector3 scaledBoundsSize = new Vector3(
            ChildMeshBounds.size.x * ChildTransform.localScale.x,
            ChildMeshBounds.size.y * ChildTransform.localScale.y,
            ChildMeshBounds.size.z * ChildTransform.localScale.z
        );
        MCollider.size = scaledBoundsSize;

        stiffnessInverse = 1 / stiffness;
        SubDt = Time.deltaTime / XPBDIterationNums;

        PointsPosition = new Vector3[8];
        PrevPointsPosition = new Vector3[8];
        PBDCorrection = new Vector3[8];
        PointsVelocity = new Vector3[8];
        PointsAcceleration = new Vector3[8];
        PointsMass = new float[8];
        PointsInvMass = new float[8];
        RestTetVolume = new float[12];
        FaceNoramls = new Vector3[6];
        
        //init connection index
        InitPointsPositions();
        InitPointConnectionIndex();
        CalculateRestLength();
        InitPhysics();
        InitBendingConstrain();
        
        int numsFaces = FaceConnectionPointIndex.GetLength(0);
        FaceNoramls = new Vector3[numsFaces];
        FaceAreas = new float[numsFaces];

        var size = MCollider.size;
        var localScale = transform.localScale;
        MappingCoef = new Vector3(size.x / localScale.x, size.y / localScale.y,
            size.z / localScale.z);
        
        //store rest volume and initial state
        RestVolume = size.x * size.y * size.z;
        InitialTransformPosition = transform.position;
        InitialTransformScale = transform.localScale;

        InitialOffset = transform.position - worldBoundsCenter;
    }

    // Update is called once per frame
    void Update()
    {
        for (int i = 0; i < XPBDIterationNums; i++)
        {
            PredictVelocity(SubDt);
            
            SolveDistanceConstrian(SubDt);
            
            SolveBendingConstrian(SubDt);
            
            SolveVolumeConstrain(SubDt);
            
            UpdatePos();
            
            CalculateVelocity(SubDt);
        }

        for (int i = 0; i < 8; i++)
        {
            PointsVelocity[i] *= 0.95f;
        }
    }

    private void InitPhysics()
    {
        //init particle mass
        for (int i = 0; i < PointsInvMass.Length; i++)
        {
            PointsMass[i] = 1.0f;
            PointsInvMass[i] = 1 / PointsMass[i];
            PointsAcceleration[i] = Physics.gravity;
        }
    }
    
    private void InitPointsPositions()
    {
        Bounds childMeshBounds = ChildMeshBounds;

        //child mesh local position
        Vector3[] corners = new Vector3[8];
        corners[0] = childMeshBounds.center + new Vector3(childMeshBounds.size.x, childMeshBounds.size.y, childMeshBounds.size.z) / 2;
        corners[1] = childMeshBounds.center + new Vector3(childMeshBounds.size.x, childMeshBounds.size.y, -childMeshBounds.size.z) / 2;
        corners[2] = childMeshBounds.center + new Vector3(-childMeshBounds.size.x, childMeshBounds.size.y, -childMeshBounds.size.z) / 2;
        corners[3] = childMeshBounds.center + new Vector3(-childMeshBounds.size.x, childMeshBounds.size.y, childMeshBounds.size.z) / 2;
        corners[4] = childMeshBounds.center + new Vector3(childMeshBounds.size.x, -childMeshBounds.size.y, childMeshBounds.size.z) / 2;
        corners[5] = childMeshBounds.center + new Vector3(childMeshBounds.size.x, -childMeshBounds.size.y, -childMeshBounds.size.z) / 2;
        corners[6] = childMeshBounds.center + new Vector3(-childMeshBounds.size.x, -childMeshBounds.size.y, -childMeshBounds.size.z) / 2;
        corners[7] = childMeshBounds.center + new Vector3(-childMeshBounds.size.x, -childMeshBounds.size.y, childMeshBounds.size.z) / 2;
        
        //local position to world position
        for (int i = 0; i < 8; i++)
        {
            corners[i] = ChildTransform.TransformPoint(corners[i]);
        }

        PointsPosition = corners;
    }
    
    private void InitPointConnectionIndex()
    {
        FaceConnectionPointIndex =
            new int[6, 4]
            { {3, 2, 1, 0}, {1, 5, 4, 0}, {2, 6, 5, 1}, {3, 7, 6, 2},
                {0, 4, 7, 3}, {4, 5, 6, 7} };

        DiagonalEdge =
            new int[32, 2]
            {
                // crossing inside hull
                { 0, 6 }, { 6, 0 }, { 1, 7 }, { 7, 1 }, { 2, 4 }, { 4, 2 }, { 3, 5 }, { 5, 3 },

                // diagonals on hull
                { 3, 1 }, { 1, 3 },
                { 2, 0 }, { 0, 2 },
                { 0, 5 }, { 5, 0 },
                { 4, 1 }, { 1, 4 },
                { 1, 6 }, { 6, 1 },
                { 5, 2 }, { 2, 5 },
                { 2, 7 }, { 7, 2 },
                { 6, 3 }, { 3, 6 },
                { 3, 4 }, { 4, 3 },
                { 7, 0 }, { 0, 7 },
                { 4, 6 }, { 6, 4 },
                { 5, 7 }, { 7, 5 }
            };
    }
    
    private void CalculateRestLength()
    {
        int numFaceEdgeConnections = FaceConnectionPointIndex.GetLength(0) * FaceConnectionPointIndex.GetLength(1);
        FaceConstrainRestLength = new float[numFaceEdgeConnections];

        int numDiagonalEdgeConnections = DiagonalEdge.GetLength(0);
        DiagionalConstrainRestLength = new float[numDiagonalEdgeConnections];
        
        int numFaces = FaceConnectionPointIndex.GetLength(0);
        int numPointsPerFace = FaceConnectionPointIndex.GetLength(1);
        int faceConstrainIndex = 0;
        
        for (int i = 0; i < numFaces; i++)
        {
            for (int j = 0; j < numPointsPerFace; j++)
            {
                int pt0Index = FaceConnectionPointIndex[i, j];
                int pt1Index = FaceConnectionPointIndex[i, (j + 1) % numPointsPerFace];
                FaceConstrainRestLength[faceConstrainIndex] =
                    (PointsPosition[pt0Index] - PointsPosition[pt1Index]).magnitude;
                ++faceConstrainIndex;
            }
        }
        
        for (int i = 0; i < numDiagonalEdgeConnections; i++)
        {
            int pt0Index = DiagonalEdge[i, 0];
            int pt1Index = DiagonalEdge[i, 1];
            DiagionalConstrainRestLength[i] = (PointsPosition[pt0Index] - PointsPosition[pt1Index]).magnitude;
        }

    }
    
    private void PredictVelocity(float dtS)
    {
        for (int i = 0; i < 8; i++)
        {
            PointsVelocity[i] += dtS * PointsAcceleration[i];
            PrevPointsPosition[i] = PointsPosition[i];
            PointsPosition[i] += dtS * PointsVelocity[i];
            if (PointsPosition[i].y < FloorHeight)
            {
                PointsPosition[i] = PrevPointsPosition[i];
                PointsPosition[i].y = FloorHeight;
            }
            
        }
    }

    private void SolveDistanceConstrian(float dtS)
    {
        var alpha = stiffnessInverse / (dtS * dtS);
        int numFaces = FaceConnectionPointIndex.GetLength(0);
        int numPtsPerFace = FaceConnectionPointIndex.GetLength(1);
        int faceConstrainIndex = 0;
        for (int i = 0; i < numFaces; i++)
        {
            for (int j = 0; j < numPtsPerFace; j++)
            {
                int pt0Index = FaceConnectionPointIndex[i, j];
                int pt1Index = FaceConnectionPointIndex[i, (j + 1) % numPtsPerFace];
                //get particle weight
                float w0 = PointsInvMass[pt0Index];
                float w1 = PointsInvMass[pt1Index];
                float w = w0 + w1;
                if (w == 0.0f)
                {
                    continue;
                }
                //calculate current distance
                Vector3 n = PointsPosition[pt0Index] - PointsPosition[pt1Index];
                float d = n.magnitude;
                if (d == 0.0f)
                {
                    continue;
                }
                //calculate diff direction
                n /= d;
                //constrain
                float C = d - FaceConstrainRestLength[faceConstrainIndex];
     
                float s = -C / (w + alpha);
                PBDCorrection[pt0Index] += s * w0 * n;
                PBDCorrection[pt1Index] -= s * w0 * n;
                
                ++faceConstrainIndex;
                
            }
        }
        return;
        int numDiagonalConnections = DiagonalEdge.GetLength(0);
        for (int i = 0; i < numDiagonalConnections; i++)
        {
            int pt0Index = DiagonalEdge[i, 0];
            int pt1Index = DiagonalEdge[i, 1];
            //get particle weight
            float w0 = PointsInvMass[pt0Index];
            float w1 = PointsInvMass[pt1Index];
            float w = w0 + w1;
            if (w == 0.0f)
            {
                continue;
            }
            //calculate current distance
            Vector3 n = PointsPosition[pt0Index] - PointsPosition[pt1Index];
            float d = n.magnitude;
            if (d == 0.0f)
            {
                continue;
            }
            //calculate diff direction
            n /= d;
            //constrain
            float C = d - DiagionalConstrainRestLength[i];
            float s = -C / (w + alpha);
            PBDCorrection[pt0Index] += s * w0 * n;
            PBDCorrection[pt1Index] -= s * w0 * n;
            
            // C = d - DiagionalConstrainRestLength[i];
            // s = -C / (w + alpha);
            // PointsPosition[pt1Index] += s * w0 * n;
            // PointsPosition[pt0Index] -= s * w0 * n;
        }
    }

    private void UpdatePos()
    {
        for (int i = 0; i < PointsPosition.Length; i++)
        {
            PointsPosition[i] += PBDCorrection[i];
            PBDCorrection[i] = new Vector3(0, 0, 0);
        }
    }
    private void SolveBendingConstrian(float dtS)
    {
        
    }
    
    private void SolveVolumeConstrain(float dtS)
    {
        CalculateFaceNormals();
    }

    
    
    private void InitBendingConstrain()
    {
        CalculateFaceNormals();
    }
    private void CalculateFaceNormals()
    {
        int numFaces = FaceConnectionPointIndex.GetLength(0);

        for (int i = 0; i < numFaces; i++)
        {
            int pt0Index = FaceConnectionPointIndex[i, 0];
            int pt1Index = FaceConnectionPointIndex[i, 1];
            int pt2Index = FaceConnectionPointIndex[i, 2];

            Vector3 edge1 = PointsPosition[pt1Index] - PointsPosition[pt0Index];
            Vector3 edge2 = PointsPosition[pt2Index] - PointsPosition[pt0Index];

            Vector3 normal = Vector3.Cross(edge1, edge2).normalized;

            // Store the calculated normal
            FaceNoramls[i] = normal;
        }
    }
    private void CalculateVelocity(float dtS)
    {
        for (int i = 0; i < PointsPosition.Length; i++)
        {
            PointsVelocity[i] = (PointsPosition[i] - PrevPointsPosition[i]) / dtS;
        }
        
    }

    private void OnTriggerEnter(Collider other)
    {
        Debug.Log("Softbody collision detected!!!");
    }

    private void OnTriggerStay(Collider other)
    {
        Vector3 depenetrationDir;
        float depenetrationDist;
        //calculate depenetration vector
        if (Physics.ComputePenetration(MCollider, transform.position, transform.rotation, other,
                other.transform.position, other.transform.rotation, out depenetrationDir, out depenetrationDist))
        {
            for (int i = 0; i < PointsPosition.Length; i++)
            {
                Vector3 p = PointsPosition[i];
                //if -- happens
                if (other.bounds.Contains(p))
                {
                    
                }
            }
        }
        throw new NotImplementedException();
    }

    private void OnTriggerExit(Collider other)
    {
        throw new NotImplementedException();
    }

    private void OnDrawGizmos()
    {
        if (PointsPosition == null || PointsPosition.Length < 4)
            return;

        // draw the top point masses
        Gizmos.color = Color.red;
        for (int i = 0; i < 4; ++i)
        {
            Gizmos.DrawSphere(PointsPosition[i], .1f);
        }

        // draw the bottom point masses
        Gizmos.color = Color.blue;
        for (int i = 4; i < PointsPosition.Length; ++i)
        {
            Gizmos.DrawSphere(PointsPosition[i], .1f);
        }

        // draw the springs used to form the faces of the point mass hull and draw
        // their normals
        
        for (int i = 0; i < FaceConnectionPointIndex.GetLength(0); ++i)
        {
            Vector3 a = PointsPosition[FaceConnectionPointIndex[i, 0]];
            Vector3 b = PointsPosition[FaceConnectionPointIndex[i, 1]];
            Vector3 c = PointsPosition[FaceConnectionPointIndex[i, 2]];
            Vector3 d = PointsPosition[FaceConnectionPointIndex[i, 3]];

            // face edges
            Gizmos.color = Color.white;
            Gizmos.DrawLine(a, b);
            Gizmos.DrawLine(b, c);
            Gizmos.DrawLine(c, d);
            Gizmos.DrawLine(d, a);
        }

        // draw the springs that cross inside the hull and hull faces
        Gizmos.color = Color.yellow;
        for (int i = 0; i < DiagonalEdge.GetLength(0); ++i)
        {
            int pt0Index = DiagonalEdge[i, 0];
            int pt1Index = DiagonalEdge[i, 1];
            Gizmos.DrawLine(PointsPosition[pt0Index], PointsPosition[pt1Index]);
        }
    }
    
    
}
