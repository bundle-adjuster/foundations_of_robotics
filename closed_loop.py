#!/usr/bin/env python3



from math import pi, sqrt, atan2, cos, sin

import numpy as np



import rospy

import tf

from std_msgs.msg import Empty

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist, Pose2D



class Turtlebot():

    def __init__(self):

        rospy.init_node("turtlebot_move") #initializing a ros node

        rospy.loginfo("Press Ctrl + C to terminate")

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10) #ros publisher t0 send movement commands (published to cmd_vel)

        self.rate = rospy.Rate(10)



        # reset odometry to zero

        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)

        for i in range(10):

            self.reset_pub.publish(Empty())

            self.rate.sleep()

        

        # subscribe to odometry

        self.pose = Pose2D() 

        self.logging_counter = 0

        self.trajectory = list()

        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback) #ros subscriber to receive position updates (subscribed to odom)



        try:

            self.run()

        except rospy.ROSInterruptException:

            rospy.loginfo("Action terminated.")

        finally:

            # save trajectory into csv file

            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')





    def run(self):

        # add your code here to adjust your movement based on 2D pose feedback

        # Use the Controller

        vel = Twist()

        ctr = Controller(P=1.8, D=1.5)

        desired_x = 3

        desired_y = -4

        desired_theta = atan2(desired_y,desired_x)

        ctr.set_point = desired_theta

        while (abs(self.pose.theta - desired_theta)) > 0.01 :

            ctr.set_point = desired_theta

            vel.angular.z = ctr.update(self.pose.theta)
            
            vel.linear.x = 0

            self.vel_pub.publish(vel)

            self.rate.sleep()

        vel.linear.x = 0
        
        vel.angular.z = 0
        
        self.vel_pub.publish(vel) 
        
        rospy.sleep(5)

        while ((abs(self.pose.x - desired_x))  and (abs(self.pose.y - desired_y))) > 0.01 :

            
            vel.linear.x = 0.15

            if abs((self.pose.theta - desired_theta)) > 0.01 :

                vel.angular.z = ctr.update(self.pose.theta)

            else:

                vel.angular.z = 0


            self.vel_pub.publish(vel)

            self.rate.sleep()

        vel.linear.x = 0
        
        vel.angular.z = 0

        self.vel_pub.publish(vel)

        rospy.sleep(5)

         

        desired_x = 6

        desired_y = 0

        desired_theta = atan2(desired_y - self.pose.y ,desired_x - self.pose.x)

        ctr.set_point = desired_theta

        while (abs(self.pose.theta - desired_theta)) > 0.01 :

            ctr.set_point = desired_theta

            vel.angular.z = ctr.update(self.pose.theta)
            
            vel.linear.x = 0

            self.vel_pub.publish(vel)

            self.rate.sleep()

        vel.linear.x = 0
        
        vel.angular.z = 0
        
        self.vel_pub.publish(vel) 
        
        rospy.sleep(5)

        while ((abs(self.pose.x - desired_x)) and (abs(self.pose.y - desired_y))) > 0.01 :

            
            vel.linear.x = 0.15

            if abs((self.pose.theta - desired_theta)) > 0.01:

                vel.angular.z = ctr.update(self.pose.theta)

            else:

                vel.angular.z = 0


            self.vel_pub.publish(vel)

            self.rate.sleep()

        vel.linear.x = 0
        
        vel.angular.z = 0

        self.vel_pub.publish(vel)

        rospy.sleep(5)

        desired_x = 3

        desired_y = 4

        desired_theta = atan2(desired_y - self.pose.y ,desired_x - self.pose.x)

        ctr.set_point = desired_theta

        while (abs(self.pose.theta - desired_theta)) > 0.01 :

            ctr.set_point = desired_theta

            vel.angular.z = ctr.update(self.pose.theta)
            
            vel.linear.x = 0

            self.vel_pub.publish(vel)

            self.rate.sleep()

        vel.linear.x = 0
        
        vel.angular.z = 0
        
        self.vel_pub.publish(vel) 
        
        rospy.sleep(5)

        while ((abs(self.pose.x - desired_x)) and (abs(self.pose.y - desired_y))) > 0.01 :

            
            vel.linear.x = 0.15

            if abs((self.pose.theta - desired_theta)) > 0.01 :

                vel.angular.z = ctr.update(self.pose.theta)

            else:

                vel.angular.z = 0


            self.vel_pub.publish(vel)

            self.rate.sleep()

        vel.linear.x = 0
        
        vel.angular.z = 0

        self.vel_pub.publish(vel)

        rospy.sleep(5)

        desired_x = 0

        desired_y = 0

        desired_theta = atan2(desired_y - self.pose.y ,desired_x - self.pose.x)

        ctr.set_point = desired_theta

        while (abs(self.pose.theta - desired_theta)) > 0.01 :

            ctr.set_point = desired_theta

            vel.angular.z = ctr.update(self.pose.theta)
            
            vel.linear.x = 0

            self.vel_pub.publish(vel)

            self.rate.sleep()

        vel.linear.x = 0
        
        vel.angular.z = 0
        
        self.vel_pub.publish(vel) 
        
        rospy.sleep(5)

        while ((abs(self.pose.x - desired_x)) and (abs(self.pose.y - desired_y))) > 0.01 :

            
            vel.linear.x = 0.15

            if abs((self.pose.theta - desired_theta)) > 0.01:

                vel.angular.z = ctr.update(self.pose.theta)

            else:

                vel.angular.z = 0


            self.vel_pub.publish(vel)

            self.rate.sleep()

        vel.linear.x = 0
        
        vel.angular.z = 0

        self.vel_pub.publish(vel)

        rospy.sleep(5)


        


        pass 
        



    # reads the robot position keverytime the robot moves, it obtains (x,y,theta) positions from odometry

    def odom_callback(self, msg):

        # get pose = (x, y, theta) from odometry topic

        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\

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

            # display (x, y, theta) on the terminal

            rospy.loginfo("odom: x=" + str(self.pose.x) +\

                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


# class to control the robots movement using the PD controller 

# Kp - proportional gain 

# Kd - derivational gain 

# set_point - the target and previous error - stores the last error for better adjustment 
    
class Controller:

    def __init__(self, P=0.0, D=0.0, set_point=0):

        self.Kp = P

        self.Kd = D

        self.set_point = set_point # reference (desired value)

        self.previous_error = 0



    def update(self, current_value):

        # calculate P_term and D_term

        error = self.set_point - current_value

        P_term = self.Kp * error

        D_term = self.Kd * (error - self.previous_error)

        self.previous_error = error

        return P_term + D_term



    def setPoint(self, set_point):

        self.set_point = set_point

        self.previous_error = 0

    

    def setPD(self, P=0.0, D=0.0):

        self.Kp = P

        self.Kd = D
    




if __name__ == '__main__':

    whatever = Turtlebot()
    

 
