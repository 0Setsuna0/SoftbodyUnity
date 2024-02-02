using System.Collections;
using System.Collections.Generic;
using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using UnityEngine;

[BurstCompile]
public struct ExPostSolvePass : IJobParallelFor
{
    [ReadOnly] public NativeArray<float> _InvMass;
    [ReadOnly] public NativeArray<Vector3> _Pos;
    [ReadOnly] public NativeArray<Vector3> _PrevPos;
    [ReadOnly] public float _dt;

    [WriteOnly] public NativeArray<Vector3> _Vel;
    public void Execute(int index)
    {
        if(_InvMass[index] == 0.0f)
            return;
        _Vel[index] = (_Pos[index] - _PrevPos[index]) / _dt;
    }
}

[BurstCompile]
public struct ExReSetXPBDParam : IJobParallelFor
{
    [WriteOnly] public NativeArray<float> _DistanceLambda;
    public void Execute(int index)
    {
        _DistanceLambda[index] = 0.0f;
    }
}
