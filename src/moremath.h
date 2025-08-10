/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef MOREMATH_H
#define MOREMATH_H

#include <math.h>

#define R2D (180.0f / M_PI)
#define D2R (M_PI / 180.0f)

/* TODO: perhaps add a .c to avoid all the inlining */

static inline float angle_normalize_pi(float angle) {
    return angle >= 0.0f ? fmodf(angle + M_PI, 2 * M_PI) - M_PI : fmodf(angle - M_PI, 2 * M_PI) + M_PI;
}

static inline float vector_normsq(const float *v) {
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

static inline float vector_norm(const float *v) {
    return sqrtf(vector_normsq(v));
}

static inline void vector_normalize(float *v) {
    float norm = vector_norm(v);

    v[0] /= norm;
    v[1] /= norm;
    v[2] /= norm;
}

static inline void vector_add(float *v, const float *dv) {
    v[0] += dv[0];
    v[1] += dv[1];
    v[2] += dv[2];
}

static inline void vector_mult_scalar(float *v, float factor) {
    v[0] *= factor;
    v[1] *= factor;
    v[2] *= factor;
}

#define INIT_VEC(v)    {  (v)[0], (v)[1], (v)[2] }

/* w, x, y, z */
#define INIT_Q(q)      {  (q)[0], (q)[1], (q)[2], (q)[3] }
#define INIT_CONJ_Q(q) { -(q)[0], (q)[1], (q)[2], (q)[3] }

static inline void quaternion_mult_to(const float *q0, const float *q1, float *q_out) {
    q_out[0] = q0[0] * q1[0] - q0[1] * q1[1] - q0[2] * q1[2] - q0[3] * q1[3];
    q_out[1] = q0[0] * q1[1] + q0[1] * q1[0] + q0[2] * q1[3] - q0[3] * q1[2];
    q_out[2] = q0[0] * q1[2] - q0[1] * q1[3] + q0[2] * q1[0] + q0[3] * q1[1];
    q_out[3] = q0[0] * q1[3] + q0[1] * q1[2] - q0[2] * q1[1] + q0[3] * q1[0];
}

static inline void quaternion_mult_scalar(float *q, float factor) {
    q[0] *= factor;
    q[1] *= factor;
    q[2] *= factor;
    q[3] *= factor;
}

static inline void vector_rotate_by_quaternion(float *v, const float *q) {
    /* Compute quaternion-vector product: q * [0, v] * q_conj */
    /* Intermediate terms for efficiency */
    float tx = 2 * (q[2] * v[2] - q[3] * v[1]);
    float ty = 2 * (q[3] * v[0] - q[1] * v[2]);
    float tz = 2 * (q[1] * v[1] - q[2] * v[0]);

    v[0] += q[0] * tx + q[2] * tz - q[3] * ty;
    v[1] += q[0] * ty + q[3] * tx - q[1] * tz;
    v[2] += q[0] * tz + q[1] * ty - q[2] * tx;
}

static inline void vector_mult_matrix(float *v, const float r[][3]) {
    float vo[3] = INIT_VEC(v);

    v[0] = r[0][0] * vo[0] + r[0][1] * vo[1] + r[0][2] * vo[2];
    v[1] = r[1][0] * vo[0] + r[1][1] * vo[1] + r[1][2] * vo[2];
    v[2] = r[2][0] * vo[0] + r[2][1] * vo[1] + r[2][2] * vo[2];
}

static inline void vector_rotate_around_axis(float *v, const float *axis, float angle) {
    float cosa = cosf(angle);
    float sina = sinf(angle);
    float vo[3] = INIT_VEC(v);
    float factor = (v[0] * axis[0] + v[1] * axis[1] + v[2] * axis[2]) * (1.0f - cosa);

    /* Rodrigues' formula */
    v[0] = vo[0] * cosa + (axis[1] * vo[2] - axis[2] * vo[1]) * sina + axis[0] * factor;
    v[1] = vo[1] * cosa + (axis[2] * vo[0] - axis[0] * vo[2]) * sina + axis[1] * factor;
    v[2] = vo[2] * cosa + (axis[0] * vo[1] - axis[1] * vo[0]) * sina + axis[2] * factor;
}

static inline float quaternion_norm(const float *q) {
    return sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

static inline void quaternion_normalize(float *q) {
    float norm = quaternion_norm(q);

    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
}

/*
 * We call these Euler angles locally but they're actually Tait-Bryan angles.
 * As a convention we use the yaw-pitch-roll order but the rotations are
 * applied in the roll-pitch-yaw order when going from global to local reference
 * frames.  We also use the ENU global axis directions and RHS rotation directions.
 */
static inline void quaternion_to_euler(const float *q, float *ypr) {
    float sinr_cosp, cosr_cosp, sinp, siny_cosp, cosy_cosp;

    /* Roll (X-axis rotation) */
    sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
    cosr_cosp = 1.0f - 2 * (q[1] * q[1] + q[2] * q[2]);
    ypr[2] = atan2f(sinr_cosp, cosr_cosp);

    /* Pitch (Y-axis rotation) */
    sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
    if (fabsf(sinp) < 1.0f)
        ypr[1] = asinf(sinp);
    else
        ypr[1] = sinp >= 0.0f ? M_PI_2 : -M_PI_2;

    /* Yaw (Z-axis rotation) */
    siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
    cosy_cosp = 1.0f - 2 * (q[2] * q[2] + q[3] * q[3]);
    ypr[0] = atan2f(siny_cosp, cosy_cosp);
}

static inline void quaternion_from_euler(const float *ypr, float *q) {
    float cr = cosf(ypr[2] / 2);
    float sr = sinf(ypr[2] / 2);
    float cp = cosf(ypr[1] / 2);
    float sp = sinf(ypr[1] / 2);
    float cy = cosf(ypr[0] / 2);
    float sy = sinf(ypr[0] / 2);

    /* Quaternion composition: X->Y->Z (note multiplication order) */
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

static inline void quaternion_from_matrix(const float r[][3], float *q) {
    float s, d;

    /* Shepherd's method */
    if (r[0][0] > r[1][1] && r[0][0] > r[2][2]) {
        s = sqrtf(1.0f + r[0][0] - r[1][1] - r[2][2]) * 0.5f;
        d = 0.25f / s;
        q[0] = (r[2][1] - r[1][2]) * d;
        q[1] = s;
        q[2] = (r[1][0] + r[0][1]) * d;
        q[3] = (r[0][2] + r[2][0]) * d;
    } else if (r[1][1] > r[0][0] && r[1][1] > r[2][2]) {
        s = sqrtf(1.0f - r[0][0] + r[1][1] - r[2][2]) * 0.5f;
        d = 0.25f / s;
        q[0] = (r[0][2] - r[2][0]) * d;
        q[1] = (r[1][0] + r[0][1]) * d;
        q[2] = s;
        q[3] = (r[2][1] + r[1][2]) * d;
    } else if (r[2][2] > r[0][0] && r[2][2] > r[1][1]) {
        s = sqrtf(1.0f - r[0][0] - r[1][1] + r[2][2]) * 0.5f;
        d = 0.25f / s;
        q[0] = (r[1][0] - r[0][1]) * d;
        q[1] = (r[0][2] + r[2][0]) * d;
        q[2] = (r[2][1] + r[1][2]) * d;
        q[3] = s;
    } else {
        s = sqrtf(1.0f + r[0][0] + r[1][1] + r[2][2]) * 0.5f;
        d = 0.25f / s;
        q[0] = s;
        q[1] = (r[2][1] - r[1][2]) * d;
        q[2] = (r[0][2] - r[2][0]) * d;
        q[3] = (r[1][0] - r[0][1]) * d;
    }
}

static inline void quaternion_to_matrix(const float *q, float r[][3]) {
    r[0][0] = 1.0f - 2 * (q[2] * q[2] + q[3] * q[3]);
    r[0][1] = 2 * (q[1] * q[2] - q[0] * q[3]);
    r[0][2] = 2 * (q[1] * q[3] + q[0] * q[2]);
    r[1][0] = 2 * (q[1] * q[2] + q[0] * q[3]);
    r[1][1] = 1.0f - 2 * (q[1] * q[1] + q[3] * q[3]);
    r[1][2] = 2 * (q[2] * q[3] - q[0] * q[1]);
    r[2][0] = 2 * (q[1] * q[3] - q[0] * q[2]);
    r[2][1] = 2 * (q[2] * q[3] + q[0] * q[1]);
    r[2][2] = 1.0f - 2 * (q[1] * q[1] + q[2] * q[2]);
}

static inline void quaternion_to_rotvec(const float *q, float *v) {
    float sin_half_angle = vector_norm(q + 1);
    float factor = 0.0f;

    if (fabsf(sin_half_angle) > 0.0001f) {
        /* Could also use acosf(q[0]) then sinf(), may be less stable and not necessarily faster */
        /* Note the sign of half_angle is the same in both cases: always positive */
        float half_angle = atan2f(sin_half_angle, q[0]);

        /*
         * Not necessary for correctness but it ensures same output for different but equivalent
         * inputs, i.e. removes the double-cover property of the quaternions by avoiding rotvec
         * magnitudes >= Pi.  Makes this function not reversible.
         */
        if (half_angle >= M_PI_2)
            half_angle -= M_PI;

        factor = 2 * half_angle / sin_half_angle;
    }

    v[0] = factor * q[1];
    v[1] = factor * q[2];
    v[2] = factor * q[3];
}

static inline void quaternion_from_axis_angle(float *q, const float *axis, float angle) {
    float sina = sinf(angle * 0.5f);

    q[0] = cosf(angle * 0.5f);
    q[1] = sina * axis[0];
    q[2] = sina * axis[1];
    q[3] = sina * axis[2];
}

static inline bool vector_solve(const float A[][3], const float *b, float *x) {
    float m[3][4]; /* augmented matrix [A | b] */

    /* Copy A and b into augmented matrix */
    for (int i = 0; i < 3; ++i) {
        m[i][0] = A[i][0];
        m[i][1] = A[i][1];
        m[i][2] = A[i][2];
        m[i][3] = b[i];
    }

#if 0
    /* Gauss-Jordan elimination */
    for (int i = 0; i < 3; ++i) {
        /* Find pivot */
        float pivot = m[i][i];
        if (fabsf(pivot) < 1e-6f)
            return false; /* Singular matrix */

        /* Normalize row */
        for (int j = 0; j < 4; j++)
            m[i][j] /= pivot;

        /* Eliminate other rows */
        for (int k = 0; k < 3; k++) {
            float factor = m[k][i];

            if (k == i)
                continue;

            for (int j = 0; j < 4; ++j)
                m[k][j] -= factor * m[i][j];
        }
    }

    /* Extract solution */
    for (int i = 0; i < 3; ++i)
        x[i] = m[i][3];
#else
    /* Gaussian elimination with partial pivoting */
    for (int i = 0; i < 3; i++) {
        int pivot_row = i;

        for (int j = i + 1; j < 3; j++) {
            if (fabs(m[j][i]) > fabs(m[pivot_row][i]))
               pivot_row = j;
        }

        /* Swap rows (is there a gcc builtin or libc function?) */
        for (int k = 0; k < 4; k++) {
             float temp = m[i][k];
             m[i][k] = m[pivot_row][k];
             m[pivot_row][k] = temp;
        }

        /* Eliminate */
        for (int j = i + 1; j < 3; j++) {
            float factor = m[j][i] / m[i][i];
            for (int k = i; k < 4; k++)
                m[j][k] -= factor * m[i][k];
        }
    }

    /* Back-substitution */
    x[2] = m[2][3] / m[2][2];
    x[1] = (m[1][3] - m[1][2] * x[2]) / m[1][1];
    x[0] = (m[0][3] - m[0][1] * x[1] - m[0][2] * x[2]) / m[0][0];
#endif

    return true;
}

static inline void vector_solve_cholesky(const float A[][3], const float *b, float *x) {
    float L[3][3];
    float y[3];

    /* Cholesky decomposition (find L such that A == L * L^T -- if it exists...) */
    L[0][0] = sqrtf(A[0][0]);
    L[1][0] = A[1][0] / L[0][0];
    L[1][1] = sqrtf(A[1][1] - L[1][0] * L[1][0]);
    L[2][0] = A[2][0] / L[0][0];
    L[2][1] = (A[2][1] - L[2][0] * L[1][0]) / L[1][1];
    L[2][2] = sqrtf(A[2][2] - L[2][0] * L[2][0] - L[2][1] * L[2][1]);

    /* Forward substitution (solve L * y = b) */
    y[0] = b[0] / L[0][0];
    y[1] = (b[1] - L[1][0] * y[0]) / L[1][1];
    y[2] = (b[2] - L[2][0] * y[0] - L[2][1] * y[1]) / L[2][2];

    /* Back-substitution (solve L^T * x = y) */
    x[2] = y[2] / L[2][2];
    x[1] = (y[1] - L[2][1] * x[2]) / L[1][1];
    x[0] = (y[0] - L[1][0] * x[1] - L[2][0] * x[2]) / L[0][0];
}

static inline void matrix_jt_mult_j(const float j[][3], float jtj[][3]) {
    jtj[0][0] = j[0][0] * j[0][0] + j[1][0] * j[1][0] + j[2][0] * j[2][0];
    jtj[0][1] = j[0][0] * j[0][1] + j[1][0] * j[1][1] + j[2][0] * j[2][1];
    jtj[0][2] = j[0][0] * j[0][2] + j[1][0] * j[1][2] + j[2][0] * j[2][2];
    jtj[1][0] = jtj[0][1];
    jtj[1][1] = j[0][1] * j[0][1] + j[1][1] * j[1][1] + j[2][1] * j[2][1];
    jtj[1][2] = j[0][1] * j[0][2] + j[1][1] * j[1][2] + j[2][1] * j[2][2];
    jtj[2][0] = jtj[0][2];
    jtj[2][1] = jtj[1][2];
    jtj[2][2] = j[0][2] * j[0][2] + j[1][2] * j[1][2] + j[2][2] * j[2][2];
}

static inline bool matrix_invert(const float m[][3], float m_inv[][3]) {
    /* Note: could also try Cholesky decomposition here for slight benefit */
    float det =
        m[0][0] *( m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
        m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
        m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

    if (fabsf(det) < 1e-8f)
        return false; /* Singular matrix */

    det = 1.0f / det;
    m_inv[0][0] =  (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * det;
    m_inv[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * det;
    m_inv[0][2] =  (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * det;
    m_inv[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * det;
    m_inv[1][1] =  (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * det;
    m_inv[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * det;
    m_inv[2][0] =  (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * det;
    m_inv[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * det;
    m_inv[2][2] =  (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * det;
    return true;
}

#endif /* MOREMATH_H */
