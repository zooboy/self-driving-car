
from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController


import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity,brake_deadband,decel_limit,
                accel_limit,wheel_radius,wheel_base,steer_ratio,max_lat_accel,max_steer_angle):
        # TODO: Implement
        tau = 0.5
        ts  = 0.02
        self.yaw_controller = YawController(wheel_base,steer_ratio,0.1,max_lat_accel,max_steer_angle)
        self.vel_lps = LowPassFilter(tau,ts)
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.max_steer_angle = max_steer_angle
        self.last_time = rospy.get_time()
        kp = 0.4 # 0.3
        ki = 0.1 # 0.1
        kd = 0.05 #0
        mn = 0.
        mx = 0.3*self.accel_limit
        self.throttle_controller = PID(kp,ki,kd,mn,mx)


        kp_cte = 0.5
        ki_cte = 0.0
        kd_cte = 0.2
        self.cte_controller = PID(kp_cte, ki_cte, kd_cte, -max_steer_angle, max_steer_angle)


    def control(self, current_vel,dbw_enabled,linear_vel,angular_vel,cte):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        # if the car driving in manual mode , reset the pid controller to avoid integral component accumulative
        if not dbw_enabled:
            self.throttle_controller.reset()
            self.cte_controller.reset()
            return 0., 0., 0.

        current_vel = self.vel_lps.filt(current_vel)
        steering = self.yaw_controller.get_steering(linear_vel,angular_vel,current_vel)
        vel_error = linear_vel -current_vel
        self.last_vel = current_vel

        current_time = rospy.get_time()
        sample_time = current_time -self.last_time
        self.last_time = current_time

        throttle = self.throttle_controller.step(vel_error,sample_time)
        steering_cte  = self.cte_controller.step(cte, sample_time)

        #use a pid controller to ajust the steer value.
        steering = steering + steering_cte
        steering = max(min(self.max_steer_angle, steering), -self.max_steer_angle)

        brake = 0


        
        if linear_vel == 0. and current_vel< 0.1:
            throttle = 0
            brake = 411

        elif throttle <.1 and vel_error<0:
            throttle = 0
            decel =max(vel_error,self.decel_limit)
            brake = abs(decel)*self.vehicle_mass*self.wheel_radius


        return throttle,brake,steering
