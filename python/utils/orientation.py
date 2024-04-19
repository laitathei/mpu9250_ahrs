#!/usr/bin/python3
import numpy as np
import math
import quaternion

class right_hand_rule:
    """
    Right-handed coordinates system rotation
    """
    def euler_x_rotation(roll):
        """
        Convert Roll angle to Direction Cosine Matrix in right-handed coordinates system

        :param float roll: x-axis Euler angle in radians
        :returns: 
            - Rx (ndarray) - x axis rotation matrix
        """
        sin_r = np.sin(roll)
        cos_r = np.cos(roll)
        Rx = np.mat([[1, 0, 0],
                    [0, cos_r, -sin_r],
                    [0, sin_r, cos_r]])
        return Rx

    def euler_y_rotation(pitch):
        """
        Convert Pitch angle to Direction Cosine Matrix in right-handed coordinates system

        :param float pitch: y-axis Euler angle in radians
        :returns: 
            - Ry (ndarray) - y axis rotation matrix
        """
        sin_p = np.sin(pitch)
        cos_p = np.cos(pitch)
        Ry = np.mat([[cos_p, 0, sin_p],
                    [0, 1, 0],
                    [-sin_p, 0, cos_p]])
        return Ry

    def euler_z_rotation(yaw):
        """
        Convert Yaw angle to Direction Cosine Matrix in right-handed coordinates system

        :param float yaw: z-axis Euler angle in radians
        :returns: 
            - Rz (ndarray) - z axis rotation matrix
        """
        sin_y = np.sin(yaw)
        cos_y = np.cos(yaw)
        Rz = np.mat([[cos_y, -sin_y, 0],
                    [sin_y, cos_y, 0],
                    [0, 0, 1]])
        return Rz

class left_hand_rule:
    """
    Left-handed coordinates system rotation
    """
    def euler_x_rotation(roll):
        """
        Convert Roll angle to Direction Cosine Matrix in left-handed coordinates system

        :param float roll: x-axis Euler angle in radians
        :returns: 
            - Rx (ndarray) - x axis rotation matrix
        """
        sin_r = np.sin(roll)
        cos_r = np.cos(roll)
        Rx = np.mat([[1, 0, 0],
                    [0, cos_r, sin_r],
                    [0, -sin_r, cos_r]])
        return Rx

    def euler_y_rotation(pitch):
        """
        Convert Pitch angle to Direction Cosine Matrix in left-handed coordinates system

        :param float pitch: y-axis Euler angle in radians
        :returns: 
            - Ry (ndarray) - y axis rotation matrix
        """
        sin_p = np.sin(pitch)
        cos_p = np.cos(pitch)
        Ry = np.mat([[cos_p, 0, -sin_p],
                    [0, 1, 0],
                    [sin_p, 0, cos_p]])
        return Ry

    def euler_z_rotation(yaw):
        """
        Convert Yaw angle to Direction Cosine Matrix in left-handed coordinates system

        :param float yaw: z-axis Euler angle in radians
        :returns: 
            - Rz (ndarray) - z axis rotation matrix
        """
        sin_y = np.sin(yaw)
        cos_y = np.cos(yaw)
        Rz = np.mat([[cos_y, sin_y, 0],
                    [-sin_y, cos_y, 0],
                    [0, 0, 1]])
        return Rz

def quat_x_rotation(roll):
    """
    Convert Roll angle to Quaternion in right-handed coordinates system (Hamilton)

    :param float roll: x-axis Euler angle in radians
    :returns: 
        - Q (quaternion.quaternion) - quaternion in w,x,y,z sequence
    """
    w = np.cos(roll/2)
    x = np.sin(roll/2)
    y = 0
    z = 0
    Q = np.quaternion(w, x, y, z)
    return Q

def quat_y_rotation(pitch):
    """
    Convert Pitch angle to Quaternion in right-handed coordinates system (Hamilton)

    :param float pitch: y-axis Euler angle in radians
    :returns: 
        - Q (quaternion.quaternion) - quaternion in w,x,y,z sequence
    """
    w = np.cos(pitch/2)
    x = 0
    y = np.sin(pitch/2)
    z = 0
    Q = np.quaternion(w, x, y, z)
    return Q

def quat_z_rotation(yaw):
    """
    Convert Yaw angle to Quaternion in right-handed coordinates system (Hamilton)

    :param float yaw: z-axis Euler angle in radians
    :returns: 
        - Q (quaternion.quaternion) - quaternion in w,x,y,z sequence
    """
    w = np.cos(yaw/2)
    x = 0
    y = 0
    z = np.sin(yaw/2)
    Q = np.quaternion(w, x, y, z)
    return Q

def eul2dcm(roll, pitch, yaw, seq="xyz", coordinates="right"):
    """
    Convert Euler angle to Direction Cosine Matrix [1]_

    :param float roll: x-axis Euler angle in radians
    :param float pitch: y-axis Euler angle in radians
    :param float yaw: z-axis Euler angle in radians
    :param str seq: rotation sequence
    :param str coordinates: right-handed or left-handed coordinates system
    :returns: 
        - DCM (numpy.matrix) - rotation matrix

    .. Reference
    .. [1] 'Wiki <https://en.wikipedia.org/wiki/Euler_angles#Tait%E2%80%93Bryan_angles>'
    """
    if coordinates == "right":
        Rx = right_hand_rule.euler_x_rotation(roll)
        Ry = right_hand_rule.euler_y_rotation(pitch)
        Rz = right_hand_rule.euler_z_rotation(yaw)
    elif coordinates == "left":
        Rx = left_hand_rule.euler_x_rotation(roll)
        Ry = left_hand_rule.euler_y_rotation(pitch)
        Rz = left_hand_rule.euler_z_rotation(yaw)
    else:
        raise ValueError("Only have right or left-handed coordinates system")
    R_dict = {"x": Rx, "y": Ry, "z": Rz}
    DCM = R_dict[seq[0]] @ R_dict[seq[1]] @ R_dict[seq[2]]
    return DCM

def eul2quat(roll, pitch, yaw, seq="xyz"):
    """
    Convert Euler angle to Quaternion [1]_

    :param float roll: x-axis Euler angle in radians
    :param float pitch: y-axis Euler angle in radians
    :param float yaw: z-axis Euler angle in radians
    :param str seq: rotation sequence
    :returns: 
        - w (float) - Quaternion magnitude
        - x (float) - Quaternion X axis
        - y (float) - Quaternion Y axis
        - z (float) - Quaternion Z axis

    .. Reference
    .. [1] 'zhihu <https://zhuanlan.zhihu.com/p/45404840>'
    """
    Qx = quat_x_rotation(roll)
    Qy = quat_y_rotation(pitch)
    Qz = quat_z_rotation(yaw)
    Q_dict = {"x": Qx, "y": Qy, "z": Qz}
    Q = Q_dict[seq[0]] * Q_dict[seq[1]] * Q_dict[seq[2]]
    w, x, y, z = Q.w, Q.x, Q.y, Q.z
    return w, x, y, z

def dcm2eul(dcm: np.matrix, seq="xyz"):
    """
    Convert Direction Cosine Matrix with specific order to Euler angle [1]_

    :param np.matrix dcm: rotation matrix
    :param str seq: rotation sequence
    :returns: 
        - roll (float) - x-axis Euler angle in radians
        - pitch (float) - y-axis Euler angle in radians
        - yaw (float) - z-axis Euler angle in radians

    .. Reference
    .. [1] 'Wiki <https://en.wikipedia.org/wiki/Euler_angles#Tait%E2%80%93Bryan_angles>'
    """
    if round(np.linalg.det(dcm),2) != 1:
        raise ValueError("Wrong rotation matrix")
    if seq == "xzy":
        yaw = -math.asin(dcm[0,1])
        pitch = math.atan2(dcm[0,2],dcm[0,0])
        roll = math.atan2(dcm[2,1],dcm[1,1])
    elif seq == "xyz":
        yaw = -math.atan2(dcm[0,1],dcm[0,0])
        pitch = math.asin(dcm[0,2])
        roll = -math.atan2(dcm[1,2],dcm[2,2])
    elif seq == "yxz":
        yaw = math.atan2(dcm[1,0],dcm[1,1])
        pitch = math.atan2(dcm[0,2],dcm[2,2])
        roll = -math.asin(dcm[1,2])
    elif seq == "yzx":
        yaw = math.asin(dcm[1,0])
        pitch = -math.atan2(dcm[2,0],dcm[0,0])
        roll = -math.atan2(dcm[1,2],dcm[1,1])
    elif seq == "zyx":
        yaw = math.atan2(dcm[1,0],dcm[0,0])
        pitch = -math.asin(dcm[2,0])
        roll = math.atan2(dcm[2,1],dcm[2,2])
    elif seq == "zxy":
        yaw = -math.atan2(dcm[0,1],dcm[1,1])
        pitch = -math.atan2(dcm[2,0],dcm[2,2])
        roll =  math.asin(dcm[2,1])
    return roll, pitch, yaw
    
def dcm2quat(dcm: np.matrix, seq="xyz"):
    """
    Convert Direction Cosine Matrix with specific order to Quaternion

    :param np.matrix dcm: rotation matrix
    :param str seq: rotation sequence
    :returns: 
        - w (float) - Quaternion magnitude
        - x (float) - Quaternion X axis
        - y (float) - Quaternion Y axis
        - z (float) - Quaternion Z axis
    """
    if round(np.linalg.det(dcm),2) != 1:
        raise ValueError("Wrong rotation matrix")
    # convert DCM to Euler angle
    roll, pitch, yaw = dcm2eul(dcm, seq)
    # convert Euler angle to Quaternions
    w, x, y, z = eul2quat(roll, pitch, yaw, seq)
    return w, x, y, z

def quat2dcm(w, x, y, z):
    """
    Convert Quaternion to Direction Cosine Matrix [1]_

    :param float w: Quaternion magnitude
    :param float x: Quaternion X axis
    :param float y: Quaternion Y axis
    :param float z: Quaternion Z axis
    :returns: 
        - DCM (numpy.matrix) - rotation matrix

    .. Reference
    .. [1] 'Wiki <https://en.wikipedia.org/wiki/Rotation_matrix#Quaternion>'
    """
    n = w*w + x*x + y*y + z*z
    if n == 0:
        s = 0
    else:
        s = 2.0/n
    xx = x*x; wx = w*x; yy = y*y; 
    xy = x*y; wy = w*y; yz = y*z; 
    xz = x*z; wz = w*z; zz = z*z; 
    DCM = np.array([[1-s*(yy+zz), s*(xy-wz), s*(xz+wy)],
                    [s*(xy+wz), 1-s*(xx+zz), s*(yz-wx)],
                    [s*(xz-wy), s*(yz+wx), 1-s*(xx+yy)]])
    return DCM

def quat2eul(w, x, y, z, seq="xyz"):
    """
    Convert Quaternion to Euler angle

    :param float w: Quaternion magnitude
    :param float x: Quaternion X axis
    :param float y: Quaternion Y axis
    :param float z: Quaternion Z axis
    :param str seq: rotation sequence
    :returns: 
        - roll (float) - x-axis Euler angle in radians
        - pitch (float) - y-axis Euler angle in radians
        - yaw (float) - z-axis Euler angle in radians
    """
    # convert Quaternion to DCM
    DCM = quat2dcm(w, x, y, z)
    # convert DCM to Euler angle
    roll, pitch, yaw = dcm2eul(DCM, seq)
    return roll, pitch, yaw

def quat_multi(q1, q2):
    """
    Calculate the product of two quaternion multiplication [1]_ [2]_

    :param ndarray q1: quaternion
    :param ndarray q2: quaternion
    :returns: 
        - q (ndarray) - product of two quaternion multiplication

    Mathematical expression: \n
    q1 = (s1, v1) \n
    q2 = (s2, v2) \n
    q1q2 = q1 x q2 - q1 · q2 (Cross product minus Dot product) \n
         = (s1 + v1)(s2 + v2) \n
         = s1s2 + s1v2 + s2v1 + v1v2 \n
         = s1s2 + s1v2 + s2v1 + (v1 x v2 - v1 · v2) \n
         = (s1s2 - v1 · v2) + (s1v2 + s2v1 + v1 x v2) \n
         = scalar part + vector part \n

    .. Reference
    .. [1] '<https://slideplayer.com/slide/16243043/>'
    .. [2] '<https://personal.utdallas.edu/~sxb027100/dock/quaternion.html>'
    """
    s1, x1, y1, z1 = q1[0][0], q1[1][0], q1[2][0], q1[3][0]
    s2, x2, y2, z2 = q2[0][0], q2[1][0], q2[2][0], q2[3][0]
    v1 = np.array([[x1],[y1],[z1]])
    v2 = np.array([[x2],[y2],[z2]])
    dot_product = x1 * x2 + y1 * y2 + z1 * z2
    cross_product = np.array([[y1 * z2 - z1 * y2], 
                              [-(x1 * z2 - z1 * x2)], 
                              [x1 * y2 - y1 * x2]]) # i,j,k
    scalar_part = s1 * s2 - dot_product
    vector_part = s1 * v2 + s2 * v1 + cross_product
    w = scalar_part
    x = vector_part[0][0]
    y = vector_part[1][0]
    z = vector_part[2][0]
    q = np.array([[w],[x],[y],[z]])
    return q

def quat_conjugate(q):
    """
    Calculate the conjugate of Quaternion [1]_

    :param ndarray q: quaternion
    :returns: 
        - q (ndarray) - the conjugate of quaternion multiplication

    .. Reference
    .. [1] 'Wiki <https://en.wikipedia.org/wiki/Quaternion#Conjugation,_the_norm,_and_reciprocal>'
    """
    w = q[0][0]
    x = q[1][0] * -1
    y = q[2][0] * -1
    z = q[3][0] * -1
    q = np.array([[w],[x],[y],[z]])
    return q

# from scipy.spatial.transform import Rotation
# roll = 0.5
# pitch = 1
# yaw = 1.5
# sinr = np.sin(roll)
# sinp = np.sin(pitch)
# siny = np.sin(yaw)
# cosr = np.cos(roll)
# cosp = np.cos(pitch)
# cosy = np.cos(yaw)

# rot = Rotation.from_euler('yzx', [pitch, yaw, roll])
# rotation_matrix = rot.as_matrix()
# print(rotation_matrix)
# print(eul2dcm(roll, pitch, yaw, seq="xzy", coordinates="right"))
# matrix=np.array([[cosy*cosp,-siny,cosy*sinp], 
# [siny*cosp*cosr+sinp*sinr,cosy*cosr,siny*sinp*cosr-cosp*sinr],
# [siny*cosp*sinr-sinp*cosr,cosy*sinr,siny*sinp*sinr+cosp*cosr]]) 
# print(matrix)

# rot = Rotation.from_euler('zyx', [yaw, pitch, roll])
# rotation_matrix = rot.as_matrix()
# print(rotation_matrix)
# print(eul2dcm(roll, pitch, yaw, seq="xyz", coordinates="right"))
# matrix=np.array([[cosy*cosp,-siny*cosp,sinp], 
# [cosy*sinp*sinr+siny*cosr,-siny*sinp*sinr+cosy*cosr,-cosp*sinr],
# [-cosy*sinp*cosr+siny*sinr,siny*sinp*cosr+cosy*sinr,cosp*cosr]]) 
# print(matrix)

# rot = Rotation.from_euler('zxy', [yaw, roll, pitch])
# rotation_matrix = rot.as_matrix()
# print(rotation_matrix)
# print(eul2dcm(roll, pitch, yaw, seq="yxz", coordinates="right"))
# matrix=np.array([[siny*sinp*sinr+cosy*cosp,cosy*sinp*sinr-siny*cosp,sinp*cosr],
# [siny*cosr,cosy*cosr,-sinr],
# [siny*cosp*sinr-cosy*sinp,cosy*cosp*sinr+siny*sinp,cosp*cosr]])  
# print(matrix)

# rot = Rotation.from_euler('xzy', [roll, yaw, pitch])
# rotation_matrix = rot.as_matrix()
# print(rotation_matrix)
# print(eul2dcm(roll, pitch, yaw, seq="yzx", coordinates="right"))
# matrix=np.array([[cosy*cosp,-siny*cosp*cosr+sinp*sinr,siny*cosp*sinr+sinp*cosr],
# [siny,cosy*cosr,-cosy*sinr],
# [-cosy*sinp,siny*sinp*cosr+cosp*sinr,-siny*sinp*sinr+cosp*cosr]])  
# print(matrix)

# rot = Rotation.from_euler('yxz', [pitch, roll, yaw])
# rotation_matrix = rot.as_matrix()
# print(rotation_matrix)
# print(eul2dcm(roll, pitch, yaw, seq="zxy", coordinates="right"))
# matrix=np.array([[-siny*sinp*sinr+cosy*cosp,-siny*cosr,siny*cosp*sinr+cosy*sinp],
# [cosy*sinp*sinr+siny*cosp,cosy*cosr,-cosy*cosp*sinr+siny*sinp],
# [-sinp*cosr,sinr,cosp*cosr]])  
# print(matrix)

# rot = Rotation.from_euler('xyz', [roll, pitch, yaw])
# rotation_matrix = rot.as_matrix()
# print(rotation_matrix)
# print(eul2dcm(roll, pitch, yaw, seq="zyx", coordinates="right"))
# matrix=np.array([[cosy*cosp,cosy*sinp*sinr-siny*cosr,cosy*sinp*cosr+siny*sinr],
# [siny*cosp,siny*sinp*sinr+cosy*cosr,siny*sinp*cosr-cosy*sinr],
# [-sinp,cosp*sinr,cosp*cosr]])  
# print(matrix)

# rot = Rotation.from_euler('xyz', [roll, pitch, yaw])
# rotation_matrix = rot.as_matrix()
# print(rotation_matrix)
# matrix=np.array([[cosy*cosp,cosy*sinp*sinr-siny*cosr,cosy*sinp*cosr+siny*sinr],
# [siny*cosp,siny*sinp*sinr+cosy*cosr,siny*sinp*cosr-cosy*sinr],
# [-sinp,cosp*sinr,cosp*cosr]])  
# print(matrix)

# sinr = np.sin(roll/2)
# sinp = np.sin(pitch/2)
# siny = np.sin(yaw/2)
# cosr = np.cos(roll/2)
# cosp = np.cos(pitch/2)
# cosy = np.cos(yaw/2)
# y = yaw
# p = pitch
# r = roll

# rot = Rotation.from_euler('yzx', [pitch, yaw, roll])
# rot_quat = rot.as_quat()
# print(rot_quat)
# print(eul2quat(roll, pitch, yaw, seq="xzy"))
# q1=w=siny*sinp*sinr+cosy*cosp*cosr # w
# q2=x=cosy*cosp*sinr-siny*sinp*cosr # x
# q3=y=-siny*cosp*sinr+cosy*sinp*cosr # y
# q4=z=cosy*sinp*sinr+siny*cosp*cosr # z
# print(q2,q3,q4,q1)
# print(x,y,z,w)

# rot = Rotation.from_euler('zyx', [yaw, pitch, roll])
# rot_quat = rot.as_quat()
# print(rot_quat)
# print(eul2quat(roll, pitch, yaw, seq="xyz"))
# q1=w=-siny*sinp*sinr+cosy*cosp*cosr 
# q2=x=cosy*cosp*sinr+siny*sinp*cosr 
# q3=y=-siny*cosp*sinr+cosy*sinp*cosr 
# q4=z=cosy*sinp*sinr+siny*cosp*cosr 
# print(q2,q3,q4,q1)
# print(x,y,z,w)

# rot = Rotation.from_euler('zxy', [yaw, roll, pitch])
# rot_quat = rot.as_quat()
# print(rot_quat)
# print(eul2quat(roll, pitch, yaw, seq="yxz"))
# q1=w=siny*sinp*sinr+cosy*cosp*cosr 
# q2=x=siny*sinp*cosr+cosy*cosp*sinr 
# q3=y=cosy*sinp*cosr-siny*cosp*sinr 
# q4=z=-cosy*sinp*sinr+siny*cosp*cosr 
# print(q2,q3,q4,q1)
# print(x,y,z,w)

# rot = Rotation.from_euler('xzy', [roll, yaw, pitch])
# rot_quat = rot.as_quat()
# print(rot_quat)
# print(eul2quat(roll, pitch, yaw, seq="yzx"))
# q1=w=-siny*sinp*sinr+cosy*cosp*cosr 
# q2=x=siny*sinp*cosr+cosy*cosp*sinr 
# q3=y=cosy*sinp*cosr+siny*cosp*sinr 
# q4=z=-cosy*sinp*sinr+siny*cosp*cosr 
# print(q2,q3,q4,q1)
# print(x,y,z,w)

# rot = Rotation.from_euler('yxz', [pitch, roll, yaw])
# rot_quat = rot.as_quat()
# print(rot_quat)
# print(eul2quat(roll, pitch, yaw, seq="zxy"))
# q1=w=-siny*sinp*sinr+cosy*cosp*cosr 
# q2=x=-siny*sinp*cosr+cosy*cosp*sinr 
# q3=y=siny*cosp*sinr+cosy*sinp*cosr 
# q4=z=siny*cosp*cosr+cosy*sinp*sinr 
# print(q2,q3,q4,q1)
# print(x,y,z,w)

# rot = Rotation.from_euler('xyz', [roll, pitch, yaw])
# rot_quat = rot.as_quat()
# print(rot_quat)
# print(eul2quat(roll, pitch, yaw, seq="zyx"))
# q1=w=siny*sinp*sinr+cosy*cosp*cosr 
# q2=x=-siny*sinp*cosr+cosy*cosp*sinr 
# q3=y=siny*cosp*sinr+cosy*sinp*cosr 
# q4=z=siny*cosp*cosr-cosy*sinp*sinr 
# print(q2,q3,q4,q1)
# print(x,y,z,w)

# euler = Rotation.from_matrix(matrix).as_euler("yzx", degrees=False)
# print(euler)
# print(dcm2eul(matrix, seq="xzy"))

# euler = Rotation.from_matrix(matrix).as_euler("zyx", degrees=False)
# print(euler)
# print(dcm2eul(matrix, seq="xyz"))

# euler = Rotation.from_matrix(matrix).as_euler("zxy", degrees=False)
# print(euler)
# print(dcm2eul(matrix, seq="yxz"))

# euler = Rotation.from_matrix(matrix).as_euler("xzy", degrees=False)
# print(euler)
# print(dcm2eul(matrix, seq="yzx"))

# euler = Rotation.from_matrix(matrix).as_euler("yxz", degrees=False)
# print(euler)
# print(dcm2eul(matrix, seq="zxy"))

# euler = Rotation.from_matrix(matrix).as_euler("xyz", degrees=False)
# print(euler)
# print(dcm2eul(matrix, seq="zyx"))

# quat = Rotation.from_matrix(matrix).as_quat()
# print(quat)
# print(dcm2quat(matrix, seq="xzy"))
# print(dcm2quat(matrix, seq="xyz"))
# print(dcm2quat(matrix, seq="yxz"))
# print(dcm2quat(matrix, seq="yzx"))
# print(dcm2quat(matrix, seq="zxy"))
# print(dcm2quat(matrix, seq="zyx"))

# q1 = np.quaternion(1,0.2,-0.4,0.7)
# q2 = np.quaternion(0.5,0.8,-0.2,-0.3)
# print(q1*q2)
# print(q1.conjugate())
# print(q2.conjugate())
# q1 = np.array([[1],[0.2],[-0.4],[0.7]])
# q2 = np.array([[0.5],[0.8],[-0.2],[-0.3]])
# print(quat_multi(q1,q2))
# print(quat_conjugate(q1))
# print(quat_conjugate(q2))

# w = 0.5
# x = 0.8
# y = -0.2
# z = -0.3
# from scipy.spatial.transform import Rotation
# rot = Rotation.from_quat([x,y,z,w])
# rotation_matrix = rot.as_matrix()
# print(rotation_matrix)
# print(quat2dcm(w, x, y, z))

# rot = Rotation.from_quat([x,y,z,w])
# euler = rot.as_euler("yzx", degrees=False)
# print(euler)
# print(quat2eul(w, x, y, z, seq="xzy"))

# rot = Rotation.from_quat([x,y,z,w])
# euler = rot.as_euler("zyx", degrees=False)
# print(euler)
# print(quat2eul(w, x, y, z, seq="xyz"))

# rot = Rotation.from_quat([x,y,z,w])
# euler = rot.as_euler("zxy", degrees=False)
# print(euler)
# print(quat2eul(w, x, y, z, seq="yxz"))

# rot = Rotation.from_quat([x,y,z,w])
# euler = rot.as_euler("xzy", degrees=False)
# print(euler)
# print(quat2eul(w, x, y, z, seq="yzx"))

# rot = Rotation.from_quat([x,y,z,w])
# euler = rot.as_euler("yxz", degrees=False)
# print(euler)
# print(quat2eul(w, x, y, z, seq="zxy"))

# rot = Rotation.from_quat([x,y,z,w])
# euler = rot.as_euler("xyz", degrees=False)
# print(euler)
# print(quat2eul(w, x, y, z, seq="zyx"))
