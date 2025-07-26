import math
import numpy as np

def quaternion_mult(q0, q1):
    q_out = [0, 0, 0, 0]
    q_out[0] = q0[0] * q1[0] - q0[1] * q1[1] - q0[2] * q1[2] - q0[3] * q1[3]
    q_out[1] = q0[0] * q1[1] + q0[1] * q1[0] + q0[2] * q1[3] - q0[3] * q1[2]
    q_out[2] = q0[0] * q1[2] - q0[1] * q1[3] + q0[2] * q1[0] + q0[3] * q1[1]
    q_out[3] = q0[0] * q1[3] + q0[1] * q1[2] - q0[2] * q1[1] + q0[3] * q1[0]
    return q_out

def rotate_reference1(q, v):
    qv = quaternion_mult(q, [0] + list(v))
    return quaternion_mult(qv, [q[0], -q[1], -q[2], q[3]])[1:]

def rotate_reference2(q, v):
    q1 = [0, 0, 0, 0]
    q1[0] = -q[1] * v[0] - q[2] * v[1] - q[3] * v[2]
    q1[1] =  q[0] * v[0] + q[2] * v[2] - q[3] * v[1]
    q1[2] =  q[0] * v[1] - q[1] * v[2] + q[3] * v[0]
    q1[3] =  q[0] * v[2] + q[1] * v[1] - q[2] * v[0]

    v_out = [0, 0, 0]
    v_out[0] = -q1[0] * q[1] + q1[1] * q[0] - q1[2] * q[3] + q1[3] * q[2]
    v_out[1] = -q1[0] * q[2] + q1[1] * q[3] + q1[2] * q[0] - q1[3] * q[1]
    v_out[2] = -q1[0] * q[3] - q1[1] * q[2] + q1[2] * q[1] + q1[3] * q[0]
    return v_out

def rotate_optimized1(q, v):
    # Compute quaternion-vector product: q * [0, v] * q_conj
    # Intermediate terms for efficiency
    tx = 2 * (q[2] * v[2] - q[3] * v[1])
    ty = 2 * (q[3] * v[0] - q[1] * v[2])
    tz = 2 * (q[1] * v[1] - q[2] * v[0])

    v_out = [0, 0, 0]
    v_out[0] = v[0] + q[0] * tx + q[2] * tz - q[3] * ty
    v_out[1] = v[1] + q[0] * ty + q[3] * tx - q[1] * tz
    v_out[2] = v[2] + q[0] * tz + q[1] * ty - q[2] * tx
    return v_out

def q2R(q, version=1):
    if version == 1:
        return np.array([
            [1.0-2.0*(q[2]**2+q[3]**2), 2.0*(q[1]*q[2]-q[0]*q[3]), 2.0*(q[1]*q[3]+q[0]*q[2])],
            [2.0*(q[1]*q[2]+q[0]*q[3]), 1.0-2.0*(q[1]**2+q[3]**2), 2.0*(q[2]*q[3]-q[0]*q[1])],
            [2.0*(q[1]*q[3]-q[0]*q[2]), 2.0*(q[0]*q[1]+q[2]*q[3]), 1.0-2.0*(q[1]**2+q[2]**2)]])
    return np.array([
        [q[0]**2+q[1]**2-q[2]**2-q[3]**2, 2.0*(q[1]*q[2]-q[0]*q[3]), 2.0*(q[1]*q[3]+q[0]*q[2])],
        [2.0*(q[1]*q[2]+q[0]*q[3]), q[0]**2-q[1]**2+q[2]**2-q[3]**2, 2.0*(q[2]*q[3]-q[0]*q[1])],
        [2.0*(q[1]*q[3]-q[0]*q[2]), 2.0*(q[0]*q[1]+q[2]*q[3]), q[0]**2-q[1]**2-q[2]**2+q[3]**2]])

qs = [(1, 0, 0, 0), (0.707, 0.707, 0, 0), (0.707, 0, 0, 0.707), (0.707, 0.707*0.707, 0, 0.707*0.707)]
vs = [(1, 0, 0), (5.5, 0.2, 0.7), (-8, -2, 0.1), (1.3, 1.3, 1.3)]
for q in qs:
    for v in vs:
        dcm = q2R(q)
        v0 = rotate_reference1(q, v)
        v1 = rotate_reference2(q, v)
        v2 = rotate_optimized1(q, v)
        v3 = dcm@v
        print(v0, v1, v2, v3)
