"""
Twiddle implementation.

This file contains all supporting code to implement Twiddle
"""

import rospy

from pid import PID



class PIDWithTwiddle(object):
    """
    A Twiddle class backed up by a PID controller.

    This class implements the twiddle algorithm to optimize parameters
    for a PID controller. It encapsulates a PID controller that mutates
    everytime a new set of parameters that yield smaller error are found.

    Once it has found the parameters that satisfy the constraints given
    (mainly the tolerance) it stops experimenting.

    You can use this class instead of a regular PID controller.

    This implementation is based on : https://www.youtube.com/watch?v=2uQ2BSzDvXs
    """

    def __init__(self, name, kp, ki, kd, mn, mx, optimize_params=False, iterations=None, tolerance=None):
        self.name = name
        self.iter = 0
        self.mn = mn
        self.mx = mx
        self.optimize_params = optimize_params
        self.tolerance = tolerance
        self.iterations = iterations
        self.p = [kp, ki, kd]
        self.dp = [0.1, 0.0001, 0.1]
        self.pid = None
        self.error = 0
        self.current_param = -1
        self.initial_state = True
        self.best_error = 0
        self.tolerance_reached = False
        self.current_state = 0

        self.mutate_pid()

    def reset(self):
        """
        Resets the underlaying PID.
        """
        if self.pid is None:
            return

        self.error = 0
        self.iter = 0
        self.pid.reset()

    def mutate_pid(self):
        """Mutate the underlaying PID with current parameters and resets counters."""
        rospy.logwarn('[%s] mutating pid with params: kp=%f, ki=%f, kd=%f --- best_error: %f, iteration: %d',
                      self.name, self.p[0], self.p[1], self.p[2], self.best_error, self.iter)
        self.error = 0
        self.iter = 0
        self.pid = PID(kp=self.p[0], ki=self.p[1],
                       kd=self.p[2], mn=self.mn, mx=self.mx)

    def reached_tolerance(self):
        """Check wheter the desired tolerance has been reached or not."""
        if sum(self.dp) < self.tolerance:
            rospy.logwarn('[%s] Tolerance of %f achieved kp=%f, ki=%f, kd=%f with best error: %f',
                          self.name, self.tolerance, self.p[0], self.p[1], self.p[2], self.best_error)
            return True
        return False

    def step(self, error, sample_time):
        """
        Perform a step on the PID controller.
        
        In case optimization of parameters is activated, it computes the error and advances
        the state of the twiddle algorithm if an run of experiments is done.
        """
        pid_output = self.pid.step(error, sample_time)
        if not self.optimize_params or self.tolerance_reached:
            return pid_output

        self.iter += 1

        if self.iter >= self.iterations:
            self.error += error ** 2

        # are we donde with this experimentation run?
        if self.iter == 2 * self.iterations:
            self.move_state()

        return pid_output

    def advance_param(self):
        """
        Advance to the next parameter index for experimentation.

        Check if tolerance has been reached to that experimentation
        is over.
        """
        self.current_param += 1
        if self.current_param == len(self.p):
            self.tolerance_reached = self.reached_tolerance()
            self.current_param %= len(self.p)

        self.p[self.current_param] += self.dp[self.current_param]
        self.mutate_pid()

    def move_state(self):
        """Transition to the next state in the Twiddle algorithm."""
        if self.initial_state:
            self.initial_state = False
            self.best_error = self.error
            self.advance_param()
            self.current_state = 0
            rospy.logwarn('[%s] initial best error: %f, kp=%f, ki=%f, kd=%f',
                          self.name, self.best_error, self.p[0], self.p[1], self.p[2])

            return

        if self.current_state == 0:
            if self.error < self.best_error:
                rospy.logwarn('[%s] new best error (0) kp=%f, ki=%f, kd=%f --- best_error: %f, new best error: %f',
                              self.name, self.p[0], self.p[1], self.p[2], self.best_error, self.error)
                self.best_error = self.error
                self.dp[self.current_param] *= 1.1
                self.advance_param()
                self.current_state = 0
            else:
                self.p[self.current_param] -= 2 * self.dp[self.current_param]
                self.mutate_pid()
                self.current_state = 1

            return

        if self.current_state == 1:
            if self.error < self.best_error:
                rospy.logwarn('[%s] new best error (1) kp=%f, ki=%f, kd=%f --- best_error: %f, new best error: %f',
                              self.name, self.p[0], self.p[1], self.p[2], self.best_error, self.error)

                self.best_error = self.error
                self.dp[self.current_param] *= 1.1
            else:
                self.p[self.current_param] += self.dp[self.current_param]
                self.dp[self.current_param] *= 0.9

            self.advance_param()
            self.current_state = 0
