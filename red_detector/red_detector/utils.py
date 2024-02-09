import math

# 欧拉角转四元数
def eular_to_quaternion(roll, pitch, yaw):
    # roll
    sinr = math.sin(roll / 2)
    cosr = math.cos(roll / 2)
    # pitch
    sinp = math.sin(pitch / 2)
    cosp = math.cos(pitch / 2)
    # yaw
    siny = math.sin(yaw / 2)
    cosy = math.cos(yaw / 2)

    w = cosy * cosp * cosr + siny * sinp * sinr
    x = cosy * cosp * sinr - siny * sinp * cosr
    y = cosy * sinp * cosr + siny * cosp * sinr
    z = siny * cosp * cosr - cosy * sinp * sinr

    return [x, y, z, w]

# 四元数转欧拉角
def quaternion_to_eular(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - z * x))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    return [roll, pitch, yaw]

def main():
    pass