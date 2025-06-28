
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
    def __init__(self, path, num_path):
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

        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        # Parameters for control
        self.position_tolerance = 0.05  # Position tolerance in meters
        
        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory'+str(num_path)+'.csv', np.array(self.trajectory), fmt='%f', delimiter=',')

    def run(self):
        waypoints = self.path
        for i in range(len(waypoints)-1):
            self.move_to_point(waypoints[i], waypoints[i+1])

    def move_to_point(self, current_waypoint, next_waypoint):
        px_start = current_waypoint[0]
        py_start = current_waypoint[1]
        px_end = next_waypoint[0]
        py_end = next_waypoint[1]
        
        if hasattr(self, 'previous_velocity'):
            vx_start = self.previous_velocity[0]
            vy_start = self.previous_velocity[1]
        else:
            vx_start = 0
            vy_start = 0
        
        diff_x = px_end - px_start
        diff_y = py_end - py_start
        target_angle = atan2(diff_y, diff_x)
        
        vx_end = 0
        vy_end = 0
        
        distance = sqrt(diff_x**2 + diff_y**2)
        
        if distance < 0.01:  # Skip if waypoints are too close
            return
            
        desired_speed = 0.12  # Slower for better precision
        
        T = max(distance / desired_speed, 1.0)  
        
        # First align with the target direction
        self.align_robot(target_angle)
        
        coef_x = self.polynomial_time_scaling_3rd_order(px_start, vx_start, px_end, vx_end, T)
        coef_y = self.polynomial_time_scaling_3rd_order(py_start, vy_start, py_end, vy_end, T)
        
        # Main trajectory execution
        for i in range(int(12*T)):  # Extended time for better accuracy
            t = min(0.1*i, T)  # Cap at T
            
            # Desired position at time t
            px = coef_x[0] + coef_x[1]*t + coef_x[2]*t*t + coef_x[3]*t*t*t
            py = coef_y[0] + coef_y[1]*t + coef_y[2]*t*t + coef_y[3]*t*t*t
            
            # Desired velocity at time t
            vx = coef_x[1] + 2*coef_x[2]*t + 3*coef_x[3]*t*t
            vy = coef_y[1] + 2*coef_y[2]*t + 3*coef_y[3]*t*t
            
            # Calculate desired speed and heading
            v = sqrt(vx*vx + vy*vy)
            theta_desired = atan2(vy, vx)
            
            # Calculate current position error
            dx = px - self.pose.x
            dy = py - self.pose.y
            position_error = sqrt(dx*dx + dy*dy)
            
            # Calculate heading to current desired position
            theta_to_point = atan2(dy, dx)
            
            # Choose appropriate theta based on how close we are to target
            if position_error > 0.1:
                # If far from desired point, head directly to it
                theta = theta_to_point
            else:
                # If close, use the trajectory-defined heading
                theta = theta_desired
            
            # Calculate orientation error
            error = theta - self.pose.theta
            while error > pi:
                error -= 2*pi
            while error < -pi:
                error += 2*pi
            
            # Control parameters
            kp_linear = 0.5
            kp_angular = 1.5
            
            # Calculate control commands
            v_cmd = min(0.2, kp_linear * position_error + 0.05)  # Base speed + proportional component
            w_cmd = kp_angular * error
            
            # Create and publish velocity command
            vel = Twist()
            vel.linear.x = v_cmd
            vel.angular.z = w_cmd
            
            self.vel_pub.publish(vel)
            self.rate.sleep()
            
            # Check if we're close enough to the end point to move on
            if i > int(8*T):  # Only check after 80% of the trajectory time
                # Calculate error to the final target
                dx_final = px_end - self.pose.x
                dy_final = py_end - self.pose.y
                final_error = sqrt(dx_final*dx_final + dy_final*dy_final)
                
                if final_error < self.position_tolerance:
                    break
        
        # Final precise positioning
        self.precise_positioning(px_end, py_end)
        
        # Stop the robot
        vel = Twist()
        vel.linear.x = 0
        vel.angular.z = 0
        self.vel_pub.publish(vel)
        
        # Update for next segment
        self.previous_waypoint = next_waypoint
        self.previous_velocity = [0, 0]  # End with zero velocity

    def align_robot(self, target_angle):
        # Calculate the angle error
        error = target_angle - self.pose.theta
        
        # Normalize to [-pi, pi]
        while error > pi:
            error -= 2*pi
        while error < -pi:
            error += 2*pi
        
        # If error is small, don't bother aligning
        if abs(error) < 0.1:
            return
        
        # Align the robot with the target direction
        vel = Twist()
        
        # Parameters
        kp = 1.0
        max_iterations = 30
        
        for i in range(max_iterations):
            # Recalculate error
            error = target_angle - self.pose.theta
            while error > pi:
                error -= 2*pi
            while error < -pi:
                error += 2*pi
            
            # Check if aligned
            if abs(error) < 0.05:
                break
            
            # Set angular velocity
            vel.angular.z = kp * error
            
            # Publish and sleep
            self.vel_pub.publish(vel)
            self.rate.sleep()
        
        # Stop rotation
        vel.angular.z = 0
        self.vel_pub.publish(vel)

    def precise_positioning(self, target_x, target_y):
        vel = Twist()
        
        # Parameters
        max_iterations = 30
        kp_pos = 0.5
        kp_orient = 1.0
        
        for i in range(max_iterations):
            # Calculate position error
            dx = target_x - self.pose.x
            dy = target_y - self.pose.y
            distance = sqrt(dx*dx + dy*dy)
            
            # If we're close enough, break
            if distance < self.position_tolerance:
                break
            
            # Calculate desired orientation
            desired_theta = atan2(dy, dx)
            
            # Calculate orientation error
            theta_error = desired_theta - self.pose.theta
            while theta_error > pi:
                theta_error -= 2*pi
            while theta_error < -pi:
                theta_error += 2*pi
            
            # Set velocities
            vel.linear.x = min(0.1, kp_pos * distance)
            vel.angular.z = kp_orient * theta_error
            
            # Publish and sleep
            self.vel_pub.publish(vel)
            self.rate.sleep()
        
        # Stop the robot
        vel.linear.x = 0
        vel.angular.z = 0
        self.vel_pub.publish(vel)

    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        a0 = p_start
        a1 = v_start
        a2 = (3*(p_end - p_start) - (2*v_start + v_end)*T) / (T*T)
        a3 = (2*(p_start - p_end) + (v_start + v_end)*T) / (T*T*T)
        
        return [a0, a1, a2, a3]

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
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
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))

    def reset_gazebo(self):
        try:
            rospy.wait_for_service('/gazebo/reset_world', timeout=2)
            reset_world = rospy.ServiceProxy('/gazebo/reset_world', srvEmpty)
            reset_world()
            rospy.loginfo('Gazebo world reset')
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logwarn('Could not reset gazebo world: %s', str(e))


if __name__ == '__main__':
    rospy.init_node('turtlebotmove')
    rospy.loginfo("Press Ctrl + C to terminate")

    # Path definitions
    path1 = [[0,0], [0,1], [0,2], [0,3], [0,4], [1,4], [2,4], [2,3], [2,2], [3,2], [4,2], [4,1], [4,0]]
    path2 = [[0.5, 0], [0.5, -0.5], [1, -0.5], [1, 0], [1, 0.5],\
              [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],\
              [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]]

    # Execute first path
    try:
        turtlebot1 = Turtlebot(path1, 1)
        turtlebot1.reset_gazebo()
    except rospy.ROSInterruptException:
        rospy.loginfo("Path 1 execution interrupted")

    # Execute second path
    try:
        turtlebot2 = Turtlebot(path2, 2)
    except rospy.ROSInterruptException:
        rospy.loginfo("Path 2 execution interrupted")