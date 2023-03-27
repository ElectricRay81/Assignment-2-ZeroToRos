#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point

# Initialize the ROS Node
rospy.init_node("input_nav_cmd", anonymous=True)

# Create the publisher
new_coordinates_pub = rospy.Publisher("/new_coordinates", Point, queue_size = 10)

# Request for user inputs for new x and y coordinates
x = float(input("Enter the new X-coordinate where the robot will have to go to: "))
y = float(input("Enter the new Y-coordinate where the robot will have to go to: "))

# Create a point message with the new received input coordinates
point = Point()
point.x = x
point.y = y


while not rospy.is_shutdown():
# Publish the Point message to new_coordinates topic
    new_coordinates_pub.publish(point)
rospy.sleep(0.1)

