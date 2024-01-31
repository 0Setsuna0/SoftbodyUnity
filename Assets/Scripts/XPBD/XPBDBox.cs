using System;
using System.Collections;
using System.Collections.Generic;
using Common.Mathematics.LinearAlgebra;
using Unity.Mathematics;
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
    private int[,] Dihedral;
    public float[] FaceConstrainRestLength;
    public float[] DiagionalConstrainRestLength;
    public float[] RestAngle;
    
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
        RestAngle = new float[8];
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
            {
                { 3, 2, 1, 0 }, { 1, 5, 4, 0 }, { 2, 6, 5, 1 }, { 3, 7, 6, 2 },
                { 0, 4, 7, 3 }, { 4, 5, 6, 7 }
            };

        DiagonalEdge =
            new int[16, 2]
            {
                // crossing inside hull
                { 0, 6 }, { 1, 7 }, { 2, 4 }, { 3, 5 },

                // diagonals on hull
                { 3, 1 },
                { 2, 0 },
                { 0, 5 },
                { 4, 1 },
                { 1, 6 },
                { 5, 2 },
                { 2, 7 },
                { 6, 3 },
                { 3, 4 },
                { 7, 0 },
                { 4, 6 },
                { 5, 7 }
            };

        Dihedral = new int[8, 4]
        {
            { 0, 4, 1, 3 },
            { 1, 5, 2, 0 },
            { 2, 6, 3, 1 },
            { 3, 7, 0, 2 },
            { 4, 0, 7, 5 },
            { 5, 1, 4, 6 },
            { 6, 2, 5, 7 },
            { 7, 3, 6, 4 }
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
                PointsVelocity[i].y *= -0.001f;
            }
            
        }
    }

    private void SolveDistanceConstrian(float dtS)
    {
        var alpha = stiffnessInverse / (dtS * dtS);
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
        }
        
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
                float invMass0 = PointsInvMass[pt0Index];
                float invMass1 = PointsInvMass[pt1Index];
                
                float K = invMass0 + invMass1;
                if (K == 0.0f)
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
                K += alpha;
                float Kinv = 1 / K;
                float lambda = -Kinv * C;
                Vector3 pt = n * lambda;

                PointsPosition[pt0Index] += pt * invMass0;
                PointsPosition[pt1Index] -= pt * invMass1;
                
                ++faceConstrainIndex;
                
            }
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
    
    private void InitBendingConstrain()
    {
        for (int i = 0; i < 8; i++)
        {
            int id0 = Dihedral[i, 0];
            int id1 = Dihedral[i, 1];
            int id2 = Dihedral[i, 2];
            int id3 = Dihedral[i, 3];

            Vector3 p0 = PointsPosition[id0];
            Vector3 p1 = PointsPosition[id1];
            Vector3 p2 = PointsPosition[id2];
            Vector3 p3 = PointsPosition[id3];

            Vector3 n1 = Vector3.Cross((p1 - p0), (p2 - p0));
            Vector3 n2 = Vector3.Cross((p3 - p0), (p2 - p0));

            n1 /= n1.magnitude;
            n2 /= n2.magnitude;
            
            n1.Normalize();
            n2.Normalize();
            float dot = Vector3.Dot(n1, n2);
            if (dot < -1.0) dot = -1.0f;
            if (dot > 1.0) dot = 1.0f;

            RestAngle[i] = math.acos(dot);

        }
    }
    
    private void SolveBendingConstrian(float dtS)
    {
        float alpha = 0.5f;
        for (int i = 0; i < 8; i++)
        {
            Vector3 p0 = PointsPosition[Dihedral[i, 0]];
            Vector3 p1 = PointsPosition[Dihedral[i, 1]];
            Vector3 p2 = PointsPosition[Dihedral[i, 2]];
            Vector3 p3 = PointsPosition[Dihedral[i, 3]];

            float invMass = 1;

            Vector3 e = p1 - p0;
            float elen = e.magnitude;
            if(elen < 1e-9)
                continue;

            float invElen = 1 / elen;
            
            Vector3 n1 = Vector3.Cross((p1 - p0), (p2 - p0));
            Vector3 n2 = Vector3.Cross((p3 - p0), (p2 - p0));
            
            n1 /= n1.magnitude;
            n2 /= n2.magnitude;
            
            Vector3 d2 = elen * n1;
            Vector3 d3 = elen * n2;
            Vector3 d0 = Vector3.Dot(p2 - p1, e) * invElen * n1 + Vector3.Dot(p3 - p1, e) * invElen * n2;
            Vector3 d1 = Vector3.Dot(p0 - p2, e) * invElen * n1 + Vector3.Dot(p0 - p3, e) * invElen * n2;
            
            n1.Normalize();
            n2.Normalize();
            float dot = Vector3.Dot(n1, n2);
            if (dot < -1.0) dot = -1.0f;
            if (dot > 1.0) dot = 1.0f;
            float phi = math.acos(dot);

            // fast approximation
            //double phi = (-0.6981317 * dot * dot - 0.8726646) * dot + 1.570796;	

            float lambda = (d0.sqrMagnitude + d1.sqrMagnitude + d2.sqrMagnitude + d3.sqrMagnitude) * invMass;

            if (lambda == 0.0) return;

            float C = phi - RestAngle[i];
            lambda = C / lambda * alpha;
            if (Vector3.Dot(Vector3.Cross(n1, n2), e) > 0.0)
                lambda = -lambda;

            PointsPosition[Dihedral[i, 0]] += d0 * (-invMass * lambda * dtS);
            PointsPosition[Dihedral[i, 1]] += d1 * (-invMass * lambda * dtS);
            PointsPosition[Dihedral[i, 2]] += d2 * (-invMass * lambda * dtS);
            PointsPosition[Dihedral[i, 3]] += d3 * (-invMass * lambda * dtS);

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
