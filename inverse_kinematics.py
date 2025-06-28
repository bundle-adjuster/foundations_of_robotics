import numpy as np

from math import pi, cos, sin, atan, atan2, sqrt, acos



def inverse_kinematics(position):

    # input: the position of end effector [x, y, z]

    # output: joint angles [joint1, joint2, joint3]

    # add your code here to complete the computation



    link1z = 0.065

    link2z = 0.039

    link3x = 0.050

    link3z = 0.150

    link4x = 0.150

    x = position[0]

    y = position[1]

    z = position[2]

    joint1 = atan2(y,x)

    offsetz = z - link1z - link2z

    xy_coord = sqrt(x**2 + y**2)

    gammahyp = sqrt( xy_coord**2 + offsetz**2 )

    alpha = atan2(0.05,0.15)

    beta1 = acos( (0.00246 + xy_coord**2 + offsetz**2) / (0.316 * (sqrt(xy_coord**2 + offsetz**2))) )

    gamma = atan2(offsetz, xy_coord)

    joint2 = (pi/2 - alpha - beta1 - gamma)

    beta2 = acos( (0.0474 - ( xy_coord**2 + offsetz**2)) / 0.0474 )

    betatemp = pi - beta2

    joint3 = pi - alpha - pi/2 - betatemp

    return [joint1, joint2, joint3]
