"""
Two stage average filter.
"""

from collections import deque

class quick_lpf:
    def __init__(self, nT1=5, nT2=25):
        """
        Create an averaging filter with 2 stages
        """
        # Queue size
        self.nTaps1 = nT1
        self.nTaps2 = nT2
        # Limit factors
        self.limitFactor1 = 0.35
        self.limitFactor2 = 1 - self.limitFactor1
        # Create the queues to contain the data
        self.queue1 = deque(maxlen=self.nTaps1)
        self.queue2 = deque(maxlen=self.nTaps2)
        self.filterGain = 1.0


    def filter(self, new_val):
        """
        Get the next filtered value
        """
        # Feed the 1st queue
        self.queue1.append(new_val)
        # feed the 2nd queue
        self.queue2.append(new_val)
        # Add elements and apply averaging
        total_q1 = sum(self.queue1) / float(self.nTaps1)
        total_q2 = sum(self.queue2) / float(self.nTaps2)
        # Apply limiting factor
        ret_val = total_q1 * self.limitFactor1 + total_q2 * self.limitFactor2
        return ret_val * self.filterGain

    def clear(self):
        """
        Remove the past values, this initializes the filter
        """
        self.queue1.clear()
        self.queue2.clear()
