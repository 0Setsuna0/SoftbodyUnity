using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;

public class FVM : MonoBehaviour
{
    float dt = 0.0025f;
    float mass = 1;
    float stiffness_0 = 5000.0f;
    float stiffness_1 = 5000.0f;
    float damp = 0.999f;

    float uN = 0.5f;
    float uT = 0.5f;
    Vector3 gravity = new Vector3(0.0f, -10.0f, 0.0f);

    int[] Tet;
    int tet_number;         //The number of tetrahedra

    Vector3[] Force;
    Vector3[] V;
    Vector3[] X;
    int number;             //The number of vertices

    Matrix4x4[] inv_Dm;
    float[] Vref_Dm;

    //For Laplacian smoothing.
    Vector3[] V_sum;
    int[] V_num;

    SVD svd = new SVD();

    // Start is called before the first frame update
    void Start()
    {
        // FILO IO: Read the house model from files.
        // The model is from Jonathan Schewchuk's Stellar lib.
        {
            string fileContent = File.ReadAllText("Assets/house2.ele");
            string[] Strings = fileContent.Split(new char[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);

            tet_number = int.Parse(Strings[0]);
            Tet = new int[tet_number * 4];

            for (int tet = 0; tet < tet_number; tet++)
            {
                Tet[tet * 4 + 0] = int.Parse(Strings[tet * 5 + 4]) - 1;
                Tet[tet * 4 + 1] = int.Parse(Strings[tet * 5 + 5]) - 1;
                Tet[tet * 4 + 2] = int.Parse(Strings[tet * 5 + 6]) - 1;
                Tet[tet * 4 + 3] = int.Parse(Strings[tet * 5 + 7]) - 1;
            }
        }
        {
            string fileContent = File.ReadAllText("Assets/house2.node");
            string[] Strings = fileContent.Split(new char[] { ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries);
            number = int.Parse(Strings[0]);
            X = new Vector3[number];
            for (int i = 0; i < number; i++)
            {
                X[i].x = float.Parse(Strings[i * 5 + 5]) * 0.4f;
                X[i].y = float.Parse(Strings[i * 5 + 6]) * 0.4f;
                X[i].z = float.Parse(Strings[i * 5 + 7]) * 0.4f;
            }
            //Centralize the model.
            Vector3 center = Vector3.zero;
            for (int i = 0; i < number; i++) center += X[i];
            center = center / number;
            for (int i = 0; i < number; i++)
            {
                X[i] -= center;
                float temp = X[i].y;
                X[i].y = X[i].z;
                X[i].z = temp;
            }
        }
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
        Vector3[] vertices = new Vector3[tet_number * 12];
        int vertex_number = 0;
        for (int tet = 0; tet < tet_number; tet++)
        {
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];

            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];

            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];

            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
        }

        int[] triangles = new int[tet_number * 12];
        for (int t = 0; t < tet_number * 4; t++)
        {
            triangles[t * 3 + 0] = t * 3 + 0;
            triangles[t * 3 + 1] = t * 3 + 1;
            triangles[t * 3 + 2] = t * 3 + 2;
        }
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();


        V = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];

        //TODO: Need to allocate and assign inv_Dm
        inv_Dm = new Matrix4x4[tet_number];
        Vref_Dm = new float[tet_number];
        for (int tet = 0; tet < tet_number; tet++)
        {
            inv_Dm[tet] = Build_Edge_Matrix(tet).inverse;
            Vref_Dm[tet] = -1.0f / (6.0f * inv_Dm[tet].determinant);
        }

    }

    Matrix4x4 Build_Edge_Matrix(int tet)
    {
        Matrix4x4 ret = Matrix4x4.zero;
        //TODO: Need to build edge matrix here.
        Vector3[] edge = new Vector3[3];
        edge[0] = X[Tet[4 * tet]] - X[Tet[4 * tet + 1]];
        edge[1] = X[Tet[4 * tet]] - X[Tet[4 * tet + 2]];
        edge[2] = X[Tet[4 * tet]] - X[Tet[4 * tet + 3]];
        ret[0, 0] = edge[0].x;
        ret[0, 1] = edge[1].x;
        ret[0, 2] = edge[2].x;
        ret[1, 0] = edge[0].y;
        ret[1, 1] = edge[1].y;
        ret[1, 2] = edge[2].y;
        ret[2, 0] = edge[0].z;
        ret[2, 1] = edge[1].z;
        ret[2, 2] = edge[2].z;
        ret[3, 3] = 1.0f;
        return ret;
    }

    Matrix4x4 MultiMat(Matrix4x4 mat, float num)
    {
        mat[0, 0] *= num;
        mat[0, 1] *= num;
        mat[0, 2] *= num;
        mat[0, 3] *= num;
        mat[1, 0] *= num;
        mat[1, 1] *= num;
        mat[1, 2] *= num;
        mat[1, 3] *= num;
        mat[2, 0] *= num;
        mat[2, 1] *= num;
        mat[2, 2] *= num;
        mat[2, 3] *= num;
        mat[3, 0] *= num;
        mat[3, 1] *= num;
        mat[3, 2] *= num;
        mat[3, 3] *= num;
        return mat;
    }

    Matrix4x4 AddMat(Matrix4x4 mat1, Matrix4x4 mat2)
    {
        Matrix4x4 res = Matrix4x4.zero;
        res[0, 0] = mat1[0, 0] + mat2[0, 0];
        res[0, 1] = mat1[0, 1] + mat2[0, 1];
        res[0, 2] = mat1[0, 2] + mat2[0, 2];
        res[0, 3] = mat1[0, 3] + mat2[0, 3];
        res[1, 0] = mat1[1, 0] + mat2[1, 0];
        res[1, 1] = mat1[1, 1] + mat2[1, 1];
        res[1, 2] = mat1[1, 2] + mat2[1, 2];
        res[1, 3] = mat1[1, 3] + mat2[1, 3];
        res[2, 0] = mat1[2, 0] + mat2[2, 0];
        res[2, 1] = mat1[2, 1] + mat2[2, 1];
        res[2, 2] = mat1[2, 2] + mat2[2, 2];
        res[2, 3] = mat1[2, 3] + mat2[2, 3];
        res[3, 0] = mat1[3, 0] + mat2[3, 0];
        res[3, 1] = mat1[3, 1] + mat2[3, 1];
        res[3, 2] = mat1[3, 2] + mat2[3, 2];
        res[3, 3] = mat1[3, 3] + mat2[3, 3];
        return res;
    }

    void _Update()
    {
        // Jump up.
        if (Input.GetKeyDown(KeyCode.Space))
        {
            for (int i = 0; i < number; i++)
                V[i].y += 0.2f;
        }

        for (int i = 0; i < number; i++)
        {
            Force[i]= mass * gravity;
            //TODO: Add gravity to Force.
        }
        //Basic Task

        for (int tet = 0; tet < tet_number; tet++)
        {
            //TODO: Deformation Gradient
            Matrix4x4 edge = Build_Edge_Matrix(tet);
            Matrix4x4 F = edge * inv_Dm[tet];
            //TODO: Green Strain
            Matrix4x4 G = F.transpose * F;
            G[0, 0] -= 1;
            G[1, 1] -= 1;
            G[2, 2] -= 1;
            G[3, 3] -= 1;
            //TODO: Second PK Stress
            float trace = G[0, 0] + G[1, 1] + G[2, 2] + G[3, 3];
            Matrix4x4 S = AddMat(MultiMat(G, 2.0f * stiffness_0), MultiMat(Matrix4x4.identity, stiffness_1 * trace));
            Matrix4x4 P = F * S;
            //TODO: Elastic Force
            Matrix4x4 force = MultiMat(P, Vref_Dm[tet]) * inv_Dm[tet].transpose;
            Vector3 f1 = new Vector3(force[0, 0], force[1, 0], force[2, 0]);
            Vector3 f2 = new Vector3(force[0, 1], force[1, 1], force[2, 1]);
            Vector3 f3 = new Vector3(force[0, 2], force[1, 2], force[2, 2]);
            Vector3 f0 = -f1 - f2 - f3;

            Force[Tet[4 * tet + 1]] += f1;
            Force[Tet[4 * tet + 2]] += f2;
            Force[Tet[4 * tet + 3]] += f3;
            Force[Tet[4 * tet]] += f0;
        }

        //Alternative Method With SVD
        //for (int tet = 0; tet < tet_number; tet++)
        //{
        //    //Deformation Gradient
        //    Matrix4x4 edge = Build_Edge_Matrix(tet);
        //    Matrix4x4 F = edge * inv_Dm[tet];
        //    //SVD
        //    Matrix4x4 U = Matrix4x4.identity;
        //    Matrix4x4 S = Matrix4x4.identity;
        //    Matrix4x4 V = Matrix4x4.identity;
        //    svd.svd(F, ref U, ref S, ref V);
        //    //right Cauchy-Green deformation tensor
        //    float[] lambda = new float[3];
        //    lambda[0] = S[0, 0];
        //    lambda[1] = S[1, 1];
        //    lambda[2] = S[2, 2];
        //    float Ic = lambda[0] * lambda[0] + lambda[1] * lambda[1] + lambda[2] * lambda[2];
        //    float dW0 = (0.5f * stiffness_0 * (Ic - 3) + 0.5f * stiffness_1 * (lambda[1] * lambda[1] + lambda[2] * lambda[2] - 2)) * lambda[0];
        //    float dW1 = (0.5f * stiffness_0 * (Ic - 3) + 0.5f * stiffness_1 * (lambda[0] * lambda[0] + lambda[2] * lambda[2] - 2)) * lambda[1];
        //    float dW2 = (0.5f * stiffness_0 * (Ic - 3) + 0.5f * stiffness_1 * (lambda[0] * lambda[0] + lambda[1] * lambda[1] - 2)) * lambda[2];

        //    Matrix4x4 W = Matrix4x4.identity;
        //    W[0, 0] = dW0;
        //    W[1, 1] = dW1;
        //    W[2, 2] = dW1;
        //    Matrix4x4 P = U * W * V.transpose;

        //    Matrix4x4 force = MultiMat(P, Vref_Dm[tet]) * inv_Dm[tet].transpose;
        //    Vector3 f1 = new Vector3(force[0, 0], force[1, 0], force[2, 0]);
        //    Vector3 f2 = new Vector3(force[0, 1], force[1, 1], force[2, 1]);
        //    Vector3 f3 = new Vector3(force[0, 2], force[1, 2], force[2, 2]);
        //    Vector3 f0 = -f1 - f2 - f3;

        //    Force[Tet[4 * tet + 1]] += f1;
        //    Force[Tet[4 * tet + 2]] += f2;
        //    Force[Tet[4 * tet + 3]] += f3;
        //    Force[Tet[4 * tet]] += f0;
        //}
            for (int i = 0; i < number; i++)
        {
            //TODO: Update X and V here.
            V[i] = damp * (V[i] + Force[i] / mass * dt);
            X[i] = X[i] + V[i] * dt;
            //TODO: (Particle) collision with floor.
            //assign impluse method
            if (X[i].y < -3.0f)
            {
                X[i].y = -3.0f;
                if (V[i].y < 0)
                {
                    //consider dynamic friction
                    float a = Mathf.Max(1 - uT * (1 + uN) * (-V[i].y) / Mathf.Sqrt(V[i].x * V[i].x + V[i].z * V[i].z), 0);

                    V[i].y *= -uN;
                    V[i].x *= a;
                    V[i].z *= a;
                }
            }
            V_sum[i] = new Vector3(0, 0, 0);
            V_num[i] = 0;
        }

        //Laplacian smoothing.
        for (int tet = 0; tet < tet_number; tet++)
        {
            V_sum[Tet[4 * tet]] += V[Tet[4 * tet + 1]] + V[Tet[4 * tet + 2]] + V[Tet[4 * tet + 3]];
            V_num[Tet[4 * tet]] += 3;
            V_sum[Tet[4 * tet + 1]] += V[Tet[4 * tet]] + V[Tet[4 * tet + 2]] + V[Tet[4 * tet + 3]];
            V_num[Tet[4 * tet + 1]] += 3;
            V_sum[Tet[4 * tet + 2]] += V[Tet[4 * tet + 1]] + V[Tet[4 * tet]] + V[Tet[4 * tet + 3]];
            V_num[Tet[4 * tet + 2]] += 3;
            V_sum[Tet[4 * tet + 3]] += V[Tet[4 * tet + 1]] + V[Tet[4 * tet + 2]] + V[Tet[4 * tet]];
            V_num[Tet[4 * tet + 3]] += 3;
        }
        for (int i = 0; i < number; i++)
        {
            V[i] = V_sum[i] / V_num[i] * 0.4f + V[i] * 0.6f;
        }
    }

    // Update is called once per frame
    void Update()
    {
        for (int l = 0; l < 10; l++)
            _Update();

        // Dump the vertex array for rendering.
        Vector3[] vertices = new Vector3[tet_number * 12];
        int vertex_number = 0;
        for (int tet = 0; tet < tet_number; tet++)
        {
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 0]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 1]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 2]];
            vertices[vertex_number++] = X[Tet[tet * 4 + 3]];
        }
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        mesh.vertices = vertices;
        mesh.RecalculateNormals();
    }
}
