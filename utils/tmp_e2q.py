import math

class Quaternion:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

class EulerAngles:
    def __init__(self, roll, pitch, yaw):
        self.roll = roll   # X-axis rotation (applied first)
        self.pitch = pitch # Y-axis rotation (applied second)
        self.yaw = yaw     # Z-axis rotation (applied last)

def q2e1(q):
    # Roll (X-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (Y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (Z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return EulerAngles(roll, pitch, yaw)

def e2q1(euler):
    # Rotations applied in X→Y→Z order (roll→pitch→yaw)
    cr = math.cos(euler.roll * 0.5)
    sr = math.sin(euler.roll * 0.5)
    cp = math.cos(euler.pitch * 0.5)
    sp = math.sin(euler.pitch * 0.5)
    cy = math.cos(euler.yaw * 0.5)
    sy = math.sin(euler.yaw * 0.5)

    # Quaternion composition: X→Y→Z (note multiplication order)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return Quaternion(x, y, z, w)

def q2e2(q):
    #*yaw = atan2f(q[1] * q[2] - q[0] * q[3], q[0] * q[0] + q[1] * q[1] - 0.5f);
    #*pitch = -asinf(2.0f * (q[1] * q[3] + q[0] * q[2]));
    #*roll = atan2f(q[2] * q[3] - q[0] * q[1], q[0] * q[0] + q[3] * q[3] - 0.5f);
    return EulerAngles(
        math.atan2(q.y * q.z - q.w * q.x, q.w * q.w + q.z * q.z - 0.5),
        -math.asin(2.0 * (q.x * q.z + q.w * q.y)),
        math.atan2(q.x * q.y - q.w * q.z, q.w * q.w + q.x * q.x - 0.5))

def q2e3(q):
    q = Quaternion(q.x, q.y, q.z, q.w)
    return EulerAngles(
        math.atan2(q.y * q.z - q.w * q.x, q.w * q.w + q.z * q.z - 0.5),
        -math.asin(2.0 * (q.x * q.z + q.w * q.y)),
        math.atan2(q.x * q.y - q.w * q.z, q.w * q.w + q.x * q.x - 0.5))

def rotate(q, v):
    """
    Rotate a 3D vector v by a unit quaternion q.
    Args:
        q: Quaternion (x, y, z, w)
        v: Vector (x, y, z)
    Returns:
        Rotated vector (x, y, z)
    """
    # Extract quaternion components
    vx, vy, vz = v[0], v[1], v[2]

    # Compute quaternion-vector product: q * [0, v] * q_conj
    # Intermediate terms for efficiency
    tx = 2 * (q.y * vz - q.z * vy)
    ty = 2 * (q.z * vx - q.x * vz)
    tz = 2 * (q.x * vy - q.y * vx)

    # Final rotated vector components
    rx = vx + q.w * tx + (q.y * tz - q.z * ty)
    ry = vy + q.w * ty + (q.z * tx - q.x * tz)
    rz = vz + q.w * tz + (q.x * ty - q.y * tx)

    return [rx, ry, rz]

l = lambda x: (x.roll, x.pitch, x.yaw)
angs = [
        EulerAngles(0, 0, math.pi / 2),
        EulerAngles(0, 0, -math.pi / 2),
        EulerAngles(0, math.pi / 4, 0),
        EulerAngles(0, math.pi / 4, math.pi / 2),
        EulerAngles(0, math.pi / 4, -math.pi / 2),
        EulerAngles(math.pi / 3, -math.pi / 4, math.pi / 2)]
v = [5, -0.1, 0.2]
for ang in angs:
    q = e2q1(ang)
    print(l(ang), '-->\n', l(q2e1(q)), '-->\n', l(q2e3(q)))
    print(rotate(q, v))
    print()
