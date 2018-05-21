from math import sqrt
from yaw_controller import YawController
from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


def norm(xs):
    return sqrt(sum(x * x for x in xs))


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, min_speed, max_lat_accel,
                 max_steer_angle):
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed,
                                            max_lat_accel,
                                            max_steer_angle)

        self.throttle_controller = PID(kp=-0.3, ki=-0.1, kd=0.0, mn=0.1, mx=1)

    def control(self, dbw_enabled, v_actual, v_target, omega_actual,
                omega_target):
        # TODO: Change the arg, kwarg list to suit your needs
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0

        linear_velocity = v_target[0]
        angular_velocity = omega_target[2]
        current_velocity = v_actual[0]
        steer = self.yaw_controller.get_steering(linear_velocity,
                                                 angular_velocity,
                                                 current_velocity)

        throttle = self.throttle_controller.step(
            current_velocity - linear_velocity,
            0.02  # Could get this exactly from messages.
        )

        if throttle <= 0.1:
            break_ = 400
        else:
            break_ = 0

        return throttle, break_, steer
