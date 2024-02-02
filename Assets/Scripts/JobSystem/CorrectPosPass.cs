using System.Collections;
using System.Collections.Generic;
using System.ComponentModel.Composition.Hosting;
using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;
using Unity.Jobs;
using Unity.Burst;

[BurstCompile]
public struct CorrectPosPass : IJobParallelFor
{
    public NativeArray<Vector3> _Correction;

    public NativeArray<Vector3> _Pos;
    
    public void Execute(int index)
    {
        _Pos[index] += 0.25f * _Correction[index];
        _Correction[index] = new Vector3(0, 0, 0);
        if (_Pos[index].y < 0)
            _Pos[index] = new Vector3(_Pos[index].x, 0, _Pos[index].z);
    }
}
