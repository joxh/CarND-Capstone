from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        # I just port all variables from dbw_node here
        # not sure if all of them will be used.
        self.vehicle_mass = kwargs['vehicle_mass']
        self.brake_deadband = kwargs['brake_deadband']
        self.decel_limit = kwargs['decel_limit']
        self.accel_limit = kwargs['accel_limit']
        self.wheel_radius = kwargs['wheel_radius']
        self.wheel_base = kwargs['wheel_base']
        self.steer_ratio = kwargs['steer_ratio']
        self.max_lat_accel = kwargs['max_lat_accel']
        self.max_steer_angle = kwargs['max_steer_angle']


        # Create the yaw contoller
        self.yaw_ctl = YawController(self.wheel_base, self.steer_ratio, 0.1, self.max_lat_accel, self.max_steer_angle)
        
        # Create a PID controller 
        # Use PID for linear and angular controll: 
        # https://discussions.udacity.com/t/solved-compute-cte-without-car-position/364383/2?u=alanxiaoyi
        # I just used some random parameters for now
        self.pid_ctl = PID(0.1, 0.1, 0.1, mn = self.decel_limit, mx = self.accel_limit)




    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        target_linear = kwargs['target_linear']
        target_angular = kwargs['target_angular']
        current_velocity = kwargs['current_velocity']
        time_delta = 1 / kwargs['rate']
        dbw_enable = kwargs['dbw_enable']

        # Get the steer from yaw contoller:
        steer = self.yaw_ctl.get_steering(target_linear.x, target_angular.x, current_velocity.twist.linear.x)

        # use pid control to get throttle
        # We only need .x from twist since it is the ego frame, x is forward
        # reference: https://answers.ros.org/question/199866/turtlebot-linear-y-has-no-response/
        error = target_linear.x - current_velocity.twist.linear.x
        throttle = self.pid_ctl.step(error, time_delta)
        
        return 10., 0., 0.
