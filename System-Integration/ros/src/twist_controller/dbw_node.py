#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped,PoseStamped
from styx_msgs.msg import Lane, Waypoint
import tf.transformations as tft
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.init_node('dbw_node')

        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',BrakeCmd, queue_size=1)

        # TODO: Create `Controller` object
        self.controller = Controller(vehicle_mass = vehicle_mass,
                                     fuel_capacity = fuel_capacity,
                                     brake_deadband = brake_deadband,
                                     decel_limit = decel_limit,
                                     accel_limit = accel_limit,
                                     wheel_radius = wheel_radius,
                                     wheel_base = wheel_base,
                                     steer_ratio = steer_ratio,
                                     max_lat_accel = max_lat_accel,
                                     max_steer_angle = max_steer_angle)

        rospy.Subscriber('/vehicle/dbw_enabled',Bool,self.dbw_enabled_cb)
        rospy.Subscriber('/twist_cmd',TwistStamped,self.twist_cb)
        rospy.Subscriber('/current_velocity',TwistStamped,self.velocity_cb)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/final_waypoints', Lane, self.final_waypoints_cb)

        self.current_vel = None
        self.curr_ang_vel = None
        self.dbw_enabled =None
        self.linear_vel = None
        self.angular_vel = None
        self.waypoints_2d = None
        self.throttle = self.steering = self.brake = 0


        # TODO: Subscribe to all the topics you need to

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            if not None in (self.current_vel,self.linear_vel,self.angular_vel):
                cte = self.cte_calc(self.waypoints_2d,self.pose)
                self.throttle,self.brake,self.steering = self.controller.control(self.current_vel,
                                                                                 self.dbw_enabled,
                                                                                 self.linear_vel,
                                                                                 self.angular_vel,
                                                                                 cte)
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            if self.dbw_enabled :
                self.publish(self.throttle, self.brake, self.steering)
                #rospy.loginfo('good')
            rate.sleep()
    def dbw_enabled_cb(self,msg):
        self.dbw_enabled = msg
    def twist_cb(self,msg):
        self.linear_vel = msg.twist.linear.x
        self.angular_vel = msg.twist.angular.z


    def velocity_cb(self,msg):
        self.current_vel =  msg.twist.linear.x

    # As the walkthough in classroom mentioned, the car perform a little shifting to the reference trajectory
    # I calculate the CTE value to the reference trajectory in order to finetune it with a PID controller


    def cte_calc(self,waypoints_2d,current_pose):
        if waypoints_2d is not None and current_pose is not None:
            #there is some geometry calculation to deal with,firstly we need turn the orientation to
            # euler angle to get the current yaw angle which rolling with z axis
            orientation = current_pose.pose.orientation
            _, _, yaw = tft.euler_from_quaternion ([orientation.x,orientation.y,orientation.z,orientation.w])
            #then get the x and y coord from current position
            car_x = []
            car_y = []
            current_x = current_pose.pose.position.x
            current_y = current_pose.pose.position.y

            #set the polyfit points counts
            fit_counts = min(10,len(waypoints_2d))

            # transform the global coord to car's  and fit it to 3 degree polynomial
            for i in range(fit_counts):
                global_x,global_y = waypoints_2d[i]
                diff_x = global_x - current_x
                diff_y = global_y - current_y
                car_x.append(diff_x * math.cos(yaw) + diff_y*math.sin(yaw))
                car_y.append(diff_y * math.cos(yaw)- diff_x*math.sin(yaw))

            coff = np.polyfit(car_x, car_y, 3)

            return np.polyval(coff, 0.)
        else:
            return 0


    def final_waypoints_cb(self, msg):
        final_waypoints = msg.waypoints
        self.way_points_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in final_waypoints]

    def pose_cb(self, msg):
        self.pose = msg



    def publish(self, throttle, brake, steer):
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)


if __name__ == '__main__':
    DBWNode()
