using System.Collections;
using System.Collections.Generic;
using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

//this pass is used for compute gradient of C ref each particle's idx
[BurstCompile]
public struct ExSolveVolumeSubPass1 : IJobParallelFor
{
    [ReadOnly] public NativeArray<int> _Triangles;
    [ReadOnly] public NativeArray<Vector3> _Pos;

    [NativeDisableParallelForRestriction] 
    public NativeArray<Vector3> _Correction;
    public void Execute(int index)
    {
        int id0 = _Triangles[3 * index];
        int id1 = _Triangles[3 * index + 1];
        int id2 = _Triangles[3 * index + 2];

        Vector3 pos0 = _Pos[id0];
        Vector3 pos1 = _Pos[id1];
        Vector3 pos2 = _Pos[id2];

        _Correction[id0] += Vector3.Cross(pos1, pos2) / 6;
        _Correction[id1] += Vector3.Cross(pos2, pos0) / 6;
        _Correction[id2] += Vector3.Cross(pos0, pos1) / 6;
        
    }
}

//this pass is used for 
[BurstCompile]
public struct ExSolveVolumeSubPass2 : IJobParallelFor
{
    [ReadOnly] public float _DeltaLambda;
    [ReadOnly] public NativeArray<float> _InvMass;
    
    public NativeArray<Vector3> _Correction;
    public NativeArray<Vector3> _Pos;
    public void Execute(int index)
    {
        _Pos[index] += 0.25f * _InvMass[index] * _DeltaLambda * _Correction[index];
        _Correction[index] = Vector3.zero;
    }
}

