using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;

[BurstCompile]
public struct PostsolvePass : IJobParallelFor
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
