using System.Collections;
using System.Collections.Generic;
using Unity.Jobs;
using Unity.Burst;
using UnityEngine;
using Unity.Collections;
using Unity.Mathematics;

[BurstCompile]
public struct ExSolveDistancePass : IJobParallelFor
{
    [ReadOnly] public NativeArray<float> _RestLength;
    [ReadOnly] public NativeArray<int2> _EdgeIdx;
    [ReadOnly] public NativeArray<float> _InvMass;
    [ReadOnly] public NativeArray<Vector3> _Pos;
    [ReadOnly] public float _alpha;

    public NativeArray<float> _Lambda;
    [NativeDisableParallelForRestriction]
    public NativeArray<Vector3> _Correction;
    
    
    public void Execute(int index)
    {
        int id0 = _EdgeIdx[index][0];
        int id1 = _EdgeIdx[index][1];

        float invMass0 = _InvMass[id0];
        float invMass1 = _InvMass[id1];
        Vector3 p0 = _Pos[id0];
        Vector3 p1 = _Pos[id1];

        float K = invMass1 + invMass0;
        Vector3 n = p0 - p1;
        float d = n.magnitude;
        
        float C = d - _RestLength[index];
        K += _alpha;

        float Kinv = 0;
        Kinv = 1 / K;
        
        float deltaLambda = Kinv * (-C - _alpha * _Lambda[index]);
        _Lambda[index] += deltaLambda;
        Vector3 pt = n * deltaLambda;
        
        _Correction[id0] += invMass0 * pt;
        _Correction[id1] -= invMass1 * pt;
        
    }
}
