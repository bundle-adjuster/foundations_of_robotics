#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist

from math import pi



class Turtlebot():

    def __init__(self):

        rospy.init_node("turtlebot_move")

        rospy.loginfo("Press Ctrl + C to terminate")

        self.vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        self.rate = rospy.Rate(10)

        self.run()





    def run(self):

        vel = Twist()

        #for 1st way point

        vel.linear.x = 0

        vel.angular.z = - 0.0981

        for i in range(80):

            self.vel_pub.publish(vel)

            self.rate.sleep()

        vel.linear.x = 0.3

        vel.angular.z = 0

        for i in range(173):

            self.vel_pub.publish(vel)

            self.rate.sleep()

        #for 2nd way point

        vel.linear.x = 0

        vel.angular.z = 0.196

        for i in range(80):

            self.vel_pub.publish(vel)

            self.rate.sleep()

        vel.linear.x = 0.3

        vel.angular.z = 0

        for i in range(173):

            self.vel_pub.publish(vel)

            self.rate.sleep()

        #for 3rd way point

        vel.linear.x = 0

        vel.angular.z = 0.196

        for i in range(80):

            self.vel_pub.publish(vel)

            self.rate.sleep()

        vel.linear.x = 0.3

        vel.angular.z = 0

        for i in range(173):

            self.vel_pub.publish(vel)

            self.rate.sleep()

        #for 4th way point

        vel.linear.x = 0

        vel.angular.z = 0.307

        for i in range(80):

            self.vel_pub.publish(vel)

            self.rate.sleep()

        vel.linear.x = 0.3

        vel.angular.z = 0
               
        for i in range(173):

            self.vel_pub.publish(vel)

            self.rate.sleep()


        vel.linear.x = 0

        vel.angular.z = 0

        self.vel_pub.publish(vel)



if __name__ == '__main__':

    try:

        tb = Turtlebot()

    except rospy.ROSInterruptException:

        rospy.loginfo("Action terminated.")

