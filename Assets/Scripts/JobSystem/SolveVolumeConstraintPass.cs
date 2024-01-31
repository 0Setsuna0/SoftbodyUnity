using System.Collections;
using System.Collections.Generic;
using Unity.Jobs;
using Unity.Burst;
using Unity.Collections;
using UnityEngine;
using Unity.Mathematics;

[BurstCompile]
public struct SolveVolumeConstraintPass : IJobParallelFor
{
    [ReadOnly] public float _alpha;
    [ReadOnly] public NativeArray<int> _TetIdx;
    [ReadOnly] public NativeArray<Vector3> _Pos;
    [ReadOnly] public NativeArray<float> _InvMass;
    [ReadOnly] public NativeArray<float> _RestVolume;
    
    [NativeDisableParallelForRestriction]
    public NativeArray<Vector3> _Correction;
    public void Execute(int index)
    {
        float volume = GetTetVolume(index);
        int id0 = _TetIdx[4 * index];
        int id1 = _TetIdx[4 * index + 1];
        int id2 = _TetIdx[4 * index + 2];
        int id3 = _TetIdx[4 * index + 3];

        float invMass0 = _InvMass[id0];
        float invMass1 = _InvMass[id1];
        float invMass2 = _InvMass[id2];
        float invMass3 = _InvMass[id3];
        
        Vector3 grad0 = math.cross((_Pos[id1] - _Pos[id2]), (_Pos[id3] - _Pos[id2]));
        Vector3 grad1 = math.cross((_Pos[id2] - _Pos[id0]), (_Pos[id3] - _Pos[id0]));
        Vector3 grad2 = math.cross((_Pos[id0] - _Pos[id1]), (_Pos[id3] - _Pos[id1]));
        Vector3 grad3 = math.cross((_Pos[id1] - _Pos[id0]), (_Pos[id2] - _Pos[id0]));

        float K = invMass0 * grad0.sqrMagnitude + invMass1 * grad1.sqrMagnitude + invMass2 * grad2.sqrMagnitude +
                  invMass3 * grad3.sqrMagnitude;

        K += _alpha;
        if(K == 0.0)
            return;
        float Kinv = 1 / K;
        float C = volume - _RestVolume[index];
        float lambda = -Kinv * C;

        if (id0 != 200)
            _Correction[id0] += lambda * invMass0 * grad0;
        if (id1 != 200)
            _Correction[id1] += lambda * invMass1 * grad1;
        if (id2 != 200)
            _Correction[id2] += lambda * invMass2 * grad2;
        if (id3 != 200)
            _Correction[id3] += lambda * invMass3 * grad3;
    }

    private float GetTetVolume(int index)
    {
        int id0 = _TetIdx[4 * index];
        int id1 = _TetIdx[4 * index + 1];
        int id2 = _TetIdx[4 * index + 2];
        int id3 = _TetIdx[4 * index + 3];

        Vector3 temp0 = _Pos[id1] - _Pos[id0];
        Vector3 temp1 = _Pos[id2] - _Pos[id0];
        Vector3 temp2 = _Pos[id3] - _Pos[id0];
        Vector3 temp3 = math.cross(temp0, temp1);

        return math.dot(temp2, temp3) / 6;
    }
}
