#!/usr/bin/env python3

from math import sqrt, atan2, pi
import numpy as np
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D
import pandas as pd
import os

class Turtlebot():
    def __init__(self):
        # Publishers and rate.
        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        self.rate = rospy.Rate(10)

        # Reset odometry.
        for _ in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()

        # Set fixed goal [x, y].
        self.goal = np.array([6.0, 0.0])
        
        # Robot state: internal velocity and pose.
        self.vel = np.array([0.0, 0.0])
        self.pose = Pose2D()
        rospy.Subscriber("odom", Odometry, self.odom_callback)

        # Controller parameters.
        self.dt = 0.1      # time step.
        self.ksi = 1.0    # relaxation time constant.
        self.robot_radius = 0.2 # physical radius + boundary you want to give to avoid collision

        # Obstacle avoidance parameters.
        self.dhor = 2.0  # sensing radius. You should tune this parameter
        self.timehor = 5.0 # time horizon for collision avoidance. You should tune this parameter

        # set desired speed
        self.dspeed = 0.7

        # Obstacle data: for each cylinder, store position, velocity, and radius.
        self.obstacles = {
            "cylinder1": {"pos": np.array([0.0, 0.0]),
                          "vel": np.array([0.0, 0.0]),
                          "radius": 0.1},
            "cylinder2": {"pos": np.array([0.0, 0.0]),
                          "vel": np.array([0.0, 0.0]),
                          "radius": 0.1},
        }

        # trajectories to for plotting
        self.traj_c1 = []
        self.traj_c2 = []
        self.traj_robot = []

        # Subscribe to obstacle topics.
        rospy.Subscriber("/cylinder1/odometry", Odometry, self.obs1_odom_callback)
        rospy.Subscriber("/cylinder1/radius", Float32, self.obs1_radius_callback)
        rospy.Subscriber("/cylinder2/odometry", Odometry, self.obs2_odom_callback)
        rospy.Subscriber("/cylinder2/radius", Float32, self.obs2_radius_callback)

        self.run()

    def computeForces(self):
        # Compute the goal-directed force.
        pos = np.array([self.pose.x, self.pose.y])
        goal_vector = self.goal - pos
        goal_distance = np.linalg.norm(goal_vector)

        if goal_distance > 0:
            goal_direction = goal_vector / goal_distance  # Normalize to get direction
        else:
            goal_direction = np.array([0.0, 0.0])  # If already at goal, no direction

        # Goal velocity (scaled by preferred speed)
        vg = goal_direction * self.dspeed

        # Goal force
        Fg = (vg - self.vel) / self.ksi

        # Compute repulsive forces from obstacles
        F_repulsive = np.array([0.0, 0.0])
        for obstacle in self.obstacles.values():
            obstacle_pos = obstacle["pos"]
            obstacle_vel = obstacle["vel"]
            obstacle_radius = obstacle["radius"]

            # Compute time-to-collision (TTC)
            tau = self.compute_ttc(obstacle)

            if tau < self.timehor:  # If collision is predicted
                # Vector from obstacle to robot
                robot_to_obstacle = pos - obstacle_pos
                distance = np.linalg.norm(robot_to_obstacle) - self.robot_radius - obstacle_radius

                if distance > 0:  # If not already colliding
                    n_tau = robot_to_obstacle / np.linalg.norm(robot_to_obstacle)  # Unit vector
                    # Scale repulsive force based on distance to obstacle
                    F_repulsive += (max(self.timehor - tau, 0) / tau) * n_tau * (1.0 / distance)

        # Total force: prioritize goal-directed force when close to the goal
        if goal_distance < 1.0:  # If within 1 meter of the goal, prioritize goal force
            F_total = Fg + 0.2 * F_repulsive  # Reduce repulsive force influence
        else:
            F_total = Fg + 0.5 * F_repulsive  # Normal repulsive force influence

        # Debug logs
        rospy.loginfo("Goal force: %s, Repulsive force: %s, Total force: %s", Fg, F_repulsive, F_total)

        return F_total

    def compute_ttc(self, obs):
        """
        Compute time-to-collision (TTC) between the robot and an obstacle.
        Returns 0 if already colliding, or infinity if no collision is predicted.
        """
        pos = np.array([self.pose.x, self.pose.y])
        obstacle_pos = obs["pos"]
        obstacle_vel = obs["vel"]

        # Relative position and velocity
        relative_pos = obstacle_pos - pos
        relative_vel = obstacle_vel - self.vel

        # Distance between robot and obstacle
        distance = np.linalg.norm(relative_pos) - self.robot_radius - obs["radius"]

        if distance <= 0:  # Already colliding
            return 0

        # Relative speed
        relative_speed = np.linalg.norm(relative_vel)

        if relative_speed == 0:  # No relative motion
            return float('inf')

        # Time-to-collision
        tau = distance / relative_speed

        return tau

    def run(self):
        rospy.loginfo("Starting closed-loop control with obstacle avoidance and P-controller...")
        Kp_linear = 0.8   # Reduced linear gain.
        Kp_angular = 0.6  # Reduced angular gain.
        goal_threshold = 0.3  # Goal threshold set to 0.3 meters.

        # Limit the robot's linear velocity to prevent aggressive behavior.
        max_linear_speed = 0.3  # Reduced maximum linear speed (m/s)
        max_angular_speed = 0.5  # Reduced maximum angular speed (rad/s)

        while not rospy.is_shutdown():
            pos = np.array([self.pose.x, self.pose.y])
            # Stop if close enough to the goal.
            if np.linalg.norm(self.goal - pos) < goal_threshold:
                self.vel_pub.publish(Twist())
                rospy.loginfo("Goal reached. Stopping robot.")
                self.save_trajectories()
                return

            # Compute total force and update internal velocity.
            F = self.computeForces()
            # Limit the robot's speed to prevent overshooting.
            self.vel += F * self.dt
            self.vel = np.clip(self.vel, -max_linear_speed, max_linear_speed)

            # Determine desired heading.
            desired_angle = atan2(self.vel[1], self.vel[0])
            angle_error = desired_angle - self.pose.theta
            while angle_error > pi:
                angle_error -= 2 * pi
            while angle_error < -pi:
                angle_error += 2 * pi

            # Control the linear and angular velocities.
            desired_speed = np.linalg.norm(self.vel)  # Calculate the speed from velocity vector
            twist = Twist()
            twist.linear.x = Kp_linear * desired_speed
            twist.angular.z = Kp_angular * angle_error

            # Limit angular velocity to avoid oscillations.
            twist.angular.z = np.clip(twist.angular.z, -max_angular_speed, max_angular_speed)

            # Publish the velocity command.
            self.vel_pub.publish(twist)
            rospy.loginfo("Twist: linear=%.2f, angular=%.2f", twist.linear.x, twist.angular.z)
            self.rate.sleep()

    def odom_callback(self, msg):
        quat = [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w]
        (_, _, yaw) = tf.transformations.euler_from_quaternion(quat)
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        self.pose.theta = yaw

        self.traj_robot.append([rospy.Time.now().to_sec(), self.pose.x, self.pose.y])

    def obs1_odom_callback(self, msg):
        self.obstacles["cylinder1"]["pos"] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.obstacles["cylinder1"]["vel"] = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])

        self.traj_c1.append([rospy.Time.now().to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y])

    def obs1_radius_callback(self, msg):
        self.obstacles["cylinder1"]["radius"] = msg.data

    def obs2_odom_callback(self, msg):
        self.obstacles["cylinder2"]["pos"] = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.obstacles["cylinder2"]["vel"] = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y])

        self.traj_c2.append([rospy.Time.now().to_sec(), msg.pose.pose.position.x, msg.pose.pose.position.y])

    def obs2_radius_callback(self, msg):
        self.obstacles["cylinder2"]["radius"] = msg.data

    def save_trajectories(self):
        print("saving the trajectory")
        traj_robot = np.array(self.traj_robot)
        traj_c1 = np.array(self.traj_c1)
        traj_c2 = np.array(self.traj_c2)

        np.save(os.path.expanduser("~")+'/catkin_ws/src/ee144w25/src/lab7/traj_robot.npy', traj_robot)
        np.save(os.path.expanduser("~")+'/catkin_ws/src/ee144w25/src/lab7/traj_c1.npy', traj_c1)
        np.save(os.path.expanduser("~")+'/catkin_ws/src/ee144w25/src/lab7/traj_c2.npy', traj_c2)


if __name__ == '__main__':
    rospy.init_node('time_based')
    try:
        Turtlebot()
    except rospy.ROSInterruptException:
        pass