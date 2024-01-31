using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;
using Unity.Mathematics;

[BurstCompile]
public struct PresolvePass : IJobParallelFor
{
    [ReadOnly] public Vector3 _Gravity;
    [ReadOnly] public float _dt;
    [ReadOnly] public NativeArray<float> _InvMass;

    [WriteOnly] public NativeArray<Vector3> _PrevPos;

    public NativeArray<Vector3> _Vel;
    public NativeArray<Vector3> _Pos;
    
    public void Execute(int index)
    {
        if (_InvMass[index] == 0 || index == 200)
            return;

        _Vel[index] += _dt * _Gravity;
        _PrevPos[index] = _Pos[index];
        _Pos[index] += _dt * _Vel[index];
    }
}
