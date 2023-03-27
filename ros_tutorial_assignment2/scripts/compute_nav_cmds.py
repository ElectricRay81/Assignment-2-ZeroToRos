#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Point
from turtlesim.msg import Pose
from ros_tutorial_assignment2.msg import NavCommands


#Initialize global variables
xCoordinate_initial = 0.0
yCoordinate_initial = 0.0
theta_initial = 0.0
xCoordinate_new = 0.0
yCoordinate_new = 0.0
theta_new = 0.0
dX = 0.0
dY = 0.0
theta_delta = 0.0
distance = 0.0

new_coordinates_found = False
initial_coordinates_found = False

#Initialize the compute nav cmds node. This node will calculate the distance the robot needs to drive at the new orientation (theta)
rospy.init_node("compute_nav_cmds", anonymous=True)

# Create the publisher
nav_cmds_pub = rospy.Publisher("/nav_cmds", NavCommands, queue_size=10)


# Create callback function for new coordinates
def point_callback(point_msg):
    global xCoordinate_new
    global yCoordinate_new
    global theta_new
    global new_coordinates_found
    xCoordinate_new = point_msg.x
    yCoordinate_new = point_msg.y
    print("New Turtle pose: x=%f, y=%f, theta=%f" % (xCoordinate_new, yCoordinate_new, theta_new))
    new_coordinates_found = True
    compute_distance()

# Create callback function for initial coordinates
def initial_pos_callback(pose):
    global xCoordinate_initial
    global yCoordinate_initial
    global theta_initial
    global initial_coordinates_found
    xCoordinate_initial = pose.x
    yCoordinate_initial = pose.y
    theta_initial = pose.theta
    initial_coordinates_found = True
    print("Turtle pose: x=%f, y=%f, theta=%f" % (xCoordinate_initial, yCoordinate_initial, theta_initial))
    compute_distance()
    

# Method that computes the distance between the initial and new coordiantes
def compute_distance():
    global dX
    global dY
    global theta_delta
    global distance
    dX = xCoordinate_new - xCoordinate_initial                                                      # Take the difference of the x-coordinates
    dY = yCoordinate_new - yCoordinate_initial                                                      # Take the difference of the y-coordinates
    distance = math.sqrt(dX**2 + dY**2)                                                             # Apply Pythagoras to get the shortes distance beween the points
    
  
    theta_new = math.atan2(dY,dX)                                                                   # Get the angle in radians with respect to the origin, atan2 function takes care of the quadrant values!!!
    
    

    # If new coordinates and initial coordinates have been found publish the distance to travel and orientation
    if new_coordinates_found and initial_coordinates_found:
        msg = NavCommands()                                                                         #NavCommands is a custom ROS message made
        msg.distance = distance
        msg.new_theta = theta_new
        msg.initial_theta = theta_initial
        msg.x_target = xCoordinate_new
        msg.y_target = yCoordinate_new
        nav_cmds_pub.publish(msg)
        print("Turtle needs to move: dx=%f, dy=%f, new-theta=%f, driving distance=%f" % (dX, dY, theta_new, distance))


#initalize the subscriber nodes
rospy.Subscriber("/new_coordinates", Point, point_callback)
rospy.Subscriber("/turtle1/pose", Pose, initial_pos_callback)


rospy.spin()






