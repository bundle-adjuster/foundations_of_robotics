import numpy as np

from math import pi, cos, sin

import modern_robotics as mr

def forward_kinematics(joints):

    # input: joint angles [joint1, joint2, joint3]

    # output: the position of end effector [x, y, z]

    # add your code here to complete the computation

    link1z = 0.065

    link2z = 0.039

    link3x = 0.050

    link3z = 0.150

    link4x = 0.150

    joint1 = joints[0]

    joint2 = joints[1]

    joint3 = joints[2]

    
    M = np.matrix([[1.0, 0, 0, (link3x + link4x)], [0, 1.0, 0, 0], [0, 0, 1.0, (link1z + link2z + link3z)], [0, 0, 0, 1.0]])

    print("This is M:", M)

    w1 = [0, 0, 1.0]

    w2 = [0, 1.0, 0]

    w3 = [0, -1.0, 0]

    q1 = [0,0,link1z]

    q2 = [0, 0, (link1z + link2z)]

    q3 = [link3x , 0 , (link1z + link2z + link3z)]

    u1 = -np.cross(w1,q1)

    u2 = -np.cross(w2,q2)

    u3 = -np.cross(w3,q3)

    S1 = [np.float64(w1[0]), float(w1[1]), w1[2], u1[0], u1[1], u1[2]]

    S2 = [float(w2[0]), w2[1], w2[2], u2[0], u2[1], u2[2]]

    S3 = [float(w3[0]), w3[1], w3[2], u3[0], u3[1], u3[2]]

    bS1 = mr.VecTose3(S1)

    bS2 = mr.VecTose3(S2)

    bS3 = mr.VecTose3(S3)

    bS1j1 = np.dot(bS1, joint1)

    bS2j2 = np.dot(bS2, joint2)

    bS3j3 = np.dot(bS3, joint3)

    eS1 = mr.MatrixExp6(bS1j1)

    eS2 = mr.MatrixExp6(bS2j2)

    eS3 = mr.MatrixExp6(bS3j3)

    T03 = M

    T03 = np.matmul(eS3, T03)

    print("eS1*M\n"+ str(T03))

    T03 = np.matmul(eS2, T03)

    print("eS2*M\n"+ str(T03))

    T03 = np.matmul(eS1, T03)

    print("final matrix\n" + str(T03))

    x = T03.item((0,3))

    y = T03.item((1,3))

    z = T03.item((2,3))


    return [x, y, z]
