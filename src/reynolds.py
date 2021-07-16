#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Point:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class Orientation:
    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

class Vector:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def get_magnitude(self):
        return math.sqrt((self.x ** 2) + (self.y ** 2) + (self.z ** 2))

    def get_normalized_vector(self):
        mag = self.get_magnitude()

        return Vector(self.x/mag, self.y/mag, self.z/mag)


#this variable keeps the max velocity for the robot in each direction
max_velocity = 1
#keep a time varibale to limit add_other_point to only be once every second
time = 0.0
#this variable keeps track of the amount of robots in the simulation, not including the main robot
robot_count = 2
#this array holds the points for each bot that's not the main bot_two_pub
robot_coordinates = [0] * robot_count
#this array holds the orientation for each bot that's not the main bot
robot_orientations = [0] * robot_count
#this variable holds the position of the main robot
current_position = Point(0, 0, 0)
#this variable holds the current orientation of the main robots
current_orientation = Orientation(0, 0, 0, 0)

def listener():
    #initialize listener
    rospy.init_node('listener', anonymous=True)
    #launch nodes to listen for data
    rospy.Subscriber('robot1/odom', Odometry, add_odom_message, 0)
    rospy.Subscriber('robot2/odom', Odometry, add_odom_message, 1)
    rospy.Subscriber('turtlebot/odom', Odometry, save_main_odom)
    #keeps python from exiting until this node is stopped
    rospy.spin()

#this function returns the dot product of two vectors
def calculate_dot_product(vec_a, vec_b):
    return (vec_a.x * vec_b.x) + (vec_a.y * vec_b.y) + (vec_a.z * vec_b.z)

#these next functions repesent reynolds rule of cohesion
#this function takes the locations from the other robots and finds the average location
def calculate_average_location(locations):
    location_count = len(locations)
    total_x = 0
    total_y = 0
    total_z = 0
    for loc in locations:
        total_x += loc.x
        total_y += loc.y
        total_z += loc.z

    return Point((total_x/location_count), (total_y/location_count), (total_z/location_count))

#this function just saves the current location of the main robot
def save_main_odom(data):
    global current_position
    global current_orientation
    pos = data.pose.pose.position
    t_or = data.pose.pose.orientation
    current_position = Point(pos.x, pos.y, pos.z)
    current_orientation = Orientation(t_or.x, t_or.y, t_or.z, t_or.w)

def get_linear_velocities(locations):
    global current_position
    global max_velocity
    #first calculate average location of robots
    average_loc = calculate_average_location(locations)
    #Display average location
    displacement_vec = Vector(average_loc.x - current_position.x, average_loc.y - current_position.y, average_loc.z - current_position.z)
    #get normalized Vector
    normal_disp = displacement_vec.get_normalized_vector()

    return [(max_velocity * normal_disp.x), ( max_velocity * normal_disp.y),( max_velocity * normal_disp.z)]

def add_odom_message(data, index):
    global time
    global robot_coordinates
    global robot_orientations
    current_time = rospy.get_time()
    #save the position from the odom message
    pos = data.pose.pose.position
    temp_point = Point(pos.x, pos.y, pos.z)
    #save orientation from the odom add_odom_message
    t_or = data.pose.pose.orientation
    temp_orientation = Orientation(t_or.x, t_or.y, t_or.z, t_or.w)
    #add current point and  to robot_coordinates array
    robot_coordinates[index] = temp_point
    #add current orientation to array or orientations
    robot_orientations[index] = temp_orientation
    #time the function to only change position variables every second
    if ((current_time - time) > 1) and (current_time > 1):
        #calculate average orientation and change to theat average_loc
        angular_velocity_z = get_angular_velocity(robot_orientations)
        time = current_time

        #setup rospy publisher
        turtlebot_vel_pub = rospy.Publisher('turtlebot/cmd_vel', Twist, queue_size=1)
        rate = rospy.Rate(2)
        move = Twist()
        #calculate average position and mget velocities to move main bot to those locations
        linear_velocities = get_linear_velocities(robot_coordinates)

        move.linear.x = linear_velocities[0]
        move.linear.y = linear_velocities[1]
        move.linear.z = linear_velocities[2]

        move.angular.z = angular_velocity_z

        turtlebot_vel_pub.publish(move)

#thse next functions represent reynolds rule of alignment
def calculate_average_orientation(orientations):
        orientation_count = len(orientations)
        total_x = 0
        total_y = 0
        total_z = 0
        total_w = 0
        for t_or in orientations:
            total_x += t_or.x
            total_y += t_or.y
            total_z += t_or.z
            total_w += t_or.w

        return Orientation((total_x/orientation_count), (total_y/orientation_count), (total_z/orientation_count), (total_w/orientation_count))

def get_angular_velocity(orientations):
    global max_velocity
    #calculate average location of the robots
    avg_or = calculate_average_orientation(orientations)
    #get cos(theta) between the angles
    avg_orientation_list = [avg_or.x, avg_or.y, avg_or. z, avg_or.w]
    (avg_roll, avg_pitch, avg_yaw) = euler_from_quaternion(avg_orientation_list)

    c_orientation_list = [current_orientation.x, current_orientation.y, current_orientation. z, current_orientation.w]
    (c_roll, c_pitch, c_yaw) = euler_from_quaternion(c_orientation_list)

    strung = "Max vel: " + str(max_velocity) + " avg_yaw: " + str(avg_yaw) + " c_yaw" + str(c_yaw)
    rospy.loginfo(strung)

    return float(max_velocity * (avg_yaw - c_yaw))


if __name__ == '__main__':
    listener()
