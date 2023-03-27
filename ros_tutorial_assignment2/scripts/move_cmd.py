#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float64
from ros_tutorial_assignment2.msg import NavCommands

distance = 0.0
theta = 0.0
target_theta = 0.0
current_theta = 0.0
current_x = 0.0
current_y = 0.0
target_x = 0.0
target_y = 0.0
dX = 0.0
dY = 0.0

target_x_found = False
target_y_found = False
target_theta_found = False

# Initialize node
rospy.init_node("move_cmd", anonymous=True)
move_cmd_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)

move_msg = Twist()

def nav_callback(nav_msg):
    global distance
    global target_theta
    global target_x
    global target_y
    global target_x_found
    global target_y_found
    global target_theta_found

    # Receive with curstom ROS message the initial position and target values
    distance = nav_msg.distance
    target_theta = nav_msg.new_theta
    initial_theta = nav_msg.initial_theta
    target_x = nav_msg.x_target
    target_y = nav_msg.y_target

    target_theta_found = True
    target_x_found = True
    target_y_found = True
    print("Turtle needs to move: driving distance = %f target orientation = %f current orientation = %f" % (distance, target_theta, initial_theta))

def current_pos_callback(pose):
    global current_theta
    global current_x
    global current_y

    # Get current position 
    current_theta = pose.theta
    current_x = pose.x
    current_y = pose.y

rospy.Subscriber("/nav_cmds", NavCommands, nav_callback)
rospy.Subscriber("/turtle1/pose", Pose, current_pos_callback)

def move_to_destination():
    global dX
    global dY
    if target_theta_found and target_x_found and target_y_found:
        # Compute current angle to rotate
        shortest_angle = math.atan2(math.sin(target_theta - current_theta), math.cos(target_theta - current_theta))
        
        # Compute current distance to travel
        dX = target_x - current_x
        dY = target_y - current_y
        delta_distance = math.sqrt(dX**2 + dY**2)  
        # Print current delta's
        print("Current delta theta = %f " % (shortest_angle))
        print("Current delta distance = %f" % (delta_distance))

        # Set linear and angular velocity
        if delta_distance <= 0.1:
            linear_vel = 0
        else:
            # Assign a linear velocity only when angle to between current orientation and target is <= 15deg   
            if abs(shortest_angle) <= 0.262:
                linear_vel = 3 * delta_distance                 #3 is proportional gain for linear velocity (the greater the difference the faster it moves)
            else:
                linear_vel = 0

        # If target and is within 2 deg make angular velocity 0
        if abs(shortest_angle) <= 0.035:
            angular_vel = 0
        else:
            angular_vel = (1/3.14) * shortest_angle             #1/3.14 is proportional gain for angular velocity (the greater the differencethe faster it rotates)
        
        move_msg.angular.z = angular_vel
        move_msg.linear.x = linear_vel
        
        #Publish velocity message
        move_cmd_pub.publish(move_msg)
        

while not rospy.is_shutdown():
    move_to_destination()




