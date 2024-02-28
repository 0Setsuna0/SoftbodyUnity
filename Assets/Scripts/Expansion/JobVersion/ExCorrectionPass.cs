using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;

[BurstCompile]
public struct ExCorrectionPass : IJobParallelFor
{
    public NativeArray<Vector3> _Correction;

    public NativeArray<Vector3> _Pos;

    [WriteOnly] public float _stiffness;
    public void Execute(int index)
    {
        _Pos[index] +=  _stiffness * _Correction[index];
        _Correction[index] = new Vector3(0, 0, 0);
        if (_Pos[index].y < -2.0f)
            _Pos[index] = new Vector3(_Pos[index].x, -2.0f, _Pos[index].z);
    }
}
