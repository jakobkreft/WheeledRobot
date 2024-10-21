#!/usr/bin/python3
# -*- coding: utf-8 -*-
import ams
from agvapi import Agv, findLineEdges
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from amsagv_msgs.msg import LineStamped
import numpy as np

with Agv() as robot:
    # Handle velocity commands
    def handleCmdVel(msg):
        robot.setVel(msg.linear.x, msg.angular.z)

    try:
        rospy.init_node('agv')
        ns = rospy.get_namespace().lstrip('/')
        # Name of the odometry frame
        paramOdomFrameId = rospy.get_param('~odom_frame_id', '{}odom'.format(ns))
        # Name of the AGV frame
        paramAgvFrameId = rospy.get_param('~agv_frame_id', '{}agv'.format(ns))

        # Odometry publisher
        pubOdom = rospy.Publisher('odom', Odometry, queue_size=1)
        # Line sensor publisher
        pubLine = rospy.Publisher('line', LineStamped, queue_size=1)
        # Velocity commands subscriber
        subCmdVel = rospy.Subscriber('cmd_vel', Twist, handleCmdVel)

        # Line-sensor message
        msgLine = LineStamped()

        # Odometry message
        msgOdom = Odometry()
        msgOdom.header.frame_id = paramOdomFrameId
        msgOdom.child_frame_id = paramAgvFrameId

        # Odometry initial state
        x, y, phi = 0.0, 0.0, 0.0  # Robot position and orientation
        fd = 0.0  # Total traveled distance
        gamma = 0.0  # Heading angle

        rate = rospy.Rate(50)  # 50 Hz update rate

        # Initial encoder readings
        prev_encLeft, prev_encRight, prev_encHeading = robot.getEncoders()

        # Constants
        # Conversion factor from encoder counts to meters
        # Adjust these values based on your robot's specifications
        wheel_radius = 0.0325  # meters (e.g., wheel diameter of 65mm)
        encoder_resolution = 11750  # counts per 10 cm of travel
        encEnota = 0.10 / encoder_resolution  # meters per count

        # Distance between the left and right wheels (wheelbase)
        wheel_base = 0.135  # meters (adjust this value to match your robot)

        while not rospy.is_shutdown():
            t = rospy.Time.now()

            # Read sensors
            robot.readSensors()

            # Get current encoder readings
            encLeft, encRight, encHeading = robot.getEncoders()

            # Calculate change in encoder counts since last reading
            delta_encLeft = encLeft - prev_encLeft
            delta_encRight = encRight - prev_encRight

            # Update previous encoder readings
            prev_encLeft = encLeft
            prev_encRight = encRight

            # Convert encoder counts to distances
            delta_left = delta_encLeft * encEnota
            delta_right = delta_encRight * encEnota

            # Odometry calculations
            delta_s = (delta_right + delta_left) / 2.0  # Linear distance traveled
            delta_theta = (delta_right - delta_left) / wheel_base  # Change in orientation

            # Update robot's pose
            x += delta_s * np.cos(phi + delta_theta / 2.0)
            y += delta_s * np.sin(phi + delta_theta / 2.0)
            phi += delta_theta

            # Normalize the orientation angle phi to the range [-pi, pi]
            phi = (phi + np.pi) % (2 * np.pi) - np.pi

            # Update total traveled distance and heading
            fd += delta_s
            gamma = phi  # Heading angle is the robot's orientation

            # Odometry message
            msgOdom.header.stamp = t
            msgOdom.pose.pose = ams.poseToPoseMsg(x, y, phi)
            # Optionally set the twist (velocity) if you have that data
            # msgOdom.twist.twist = ...  # Fill in linear and angular velocities

            # Publish odometry message
            pubOdom.publish(msgOdom)

            #
            # Line sensor
            #

            # Line-sensor values
            lineValues = robot.getLineValues()
            # Left and right line edge
            edgeLeft, edgeRight = findLineEdges(lineValues)

            # Line-sensor message
            msgLine.header.stamp = t
            msgLine.line.values = lineValues
            msgLine.line.left = edgeLeft if edgeLeft is not None else float('nan')
            msgLine.line.right = edgeRight if edgeRight is not None else float('nan')
            msgLine.line.heading = gamma
            msgLine.line.distance = fd
            # Publish line-sensor message
            pubLine.publish(msgLine)

            rate.sleep()
    except KeyboardInterrupt:
        pass

