#!/usr/bin/env python3

from math import pi, sqrt, atan2, cos, sin
import numpy as np
from copy import deepcopy

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
from std_srvs.srv import Empty as srvEmpty


class Turtlebot():
    def __init__(self, path, num_path, wf, ws):
        # rospy.init_node(node_name)
        # rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()

        # set the path
        self.path = path
        self.wf = wf
        self.ws = ws

        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        # Initialize missing attributes
        self.previous_waypoint = [0, 0]
        self.previous_velocity = [0, 0]
        self.vel_ref = 0.2  # Default velocity reference
        self.kp = 1.0  # Default proportional gain

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory'+str(num_path)+'.csv', np.array(self.trajectory), fmt='%f', delimiter=',')

        self.reset_gazebo()

    def run(self):
        waypoints = self.path
        while True:
            if len(waypoints) == 1:
                break
            waypoints[0] = [self.pose.x, self.pose.y]

            waypoints = self.smooth(waypoints, alpha=1, weight_follow=self.wf, weight_smooth=self.ws)
            print("waypoints", waypoints[0], self.pose.x, self.pose.y)
            
            self.move_to_point(waypoints[0], waypoints[1])
            self.path = self.path[1:]
            waypoints = self.path  # Fixed typo from wayppoints to waypoints

    def move_to_point(self, current_waypoint, next_waypoint):
        px_start = self.previous_waypoint[0]
        px_end = current_waypoint[0]
        py_start = self.previous_waypoint[1]
        py_end = current_waypoint[1]

        vx_start = self.previous_velocity[0]
        vy_start = self.previous_velocity[1]
   
        dx = next_waypoint[0] - current_waypoint[0]
        dy = next_waypoint[1] - current_waypoint[1]

        angle = atan2(dy, dx)
        distance = sqrt(dx**2 + dy**2)
   
        T = distance/self.vel_ref
   
        for i in range(int(10*T)):
            vel_x = dx/T
            vel_y = dy/T
            theta = atan2(dy, dx)
            d_theta = theta - self.pose.theta
       
            while d_theta > pi:
                d_theta -= 2*pi
            while d_theta < -pi:
                d_theta += 2*pi
            
            vel_cmd = Twist()
            vel_cmd.linear.x = sqrt(vel_x**2 + vel_y**2)
            vel_cmd.angular.z = self.kp*d_theta
    
            self.vel_pub.publish(vel_cmd)
            self.rate.sleep()

    def smooth(self, path, alpha=1, weight_follow=0.5, weight_smooth=0.1, tolerance=0.000001):
        newpath = deepcopy(path)  # Make a copy of the original path 

        change = float('inf')  
        while change > tolerance:  
            change = 0
            for i in range(1, len(path) - 1):  
                for j in range(len(path[0])):  
                    aux = newpath[i][j]
                    newpath[i][j] += weight_follow * (path[i][j] - newpath[i][j])
                    newpath[i][j] += weight_smooth * (newpath[i-1][j] + newpath[i+1][j] - 2 * newpath[i][j])
                    change += abs(aux - newpath[i][j])

        return newpath

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) +
                         ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))

    def reset_gazebo(self):
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', srvEmpty)
            reset_world()
            print('resetting')
        except rospy.ServiceException as e:
            print('failed' % e)

    def printpaths(self, path, newpath):  # This will show the initial path before smoothing
        for old, new in zip(path, newpath):
            print('[' + ', '.join('%.3f' % x for x in old) +
                 '] -> [' + ', '.join('%.3f' % x for x in new) + ']')

    def plot(self, path, newpath):
        x = [x for [x, y] in path]
        y = [y for [ax, y] in path]
        # This plots the smooth path overlayed on the original 
        plt.plot(x, y, color='r', marker='o')
        x2 = [x for [x, y] in newpath]
        y2 = [y for [ax, y] in newpath]
        plt.plot(x2, y2, color='b', marker='o')
        plt.xlim(-1, 3)
        plt.ylim(-1, 3)
        plt.show()


if __name__ == '__main__':
    #path1
    path1 = [[0,0], [0,1], [0,2], [0,3], [0,4], [1,4], [2,4], [2,3], [2,2], [3,2], [4,2], [4,1], [4,0]]

    #path2
    path2 = [[0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],
             [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],
             [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]]

    rospy.init_node('turtlebotmove')
    rospy.loginfo("Press Ctrl + C to terminate")

    whatever1 = Turtlebot(path1, 1, wf=0.1, ws=0.1)
    whatever2 = Turtlebot(path2, 2, wf=0.1, ws=0.1)

