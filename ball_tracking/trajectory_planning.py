from ball_tracking import MaskBallTracker
from math import sqrt

tracker = MaskBallTracker()

path = tracker.get_path()  # Would normally be path so far when we subscribe to the ROS topic 
                           # Append to the trajectory array and predict again for updated time [t - 1] for final position

# https://people.physics.tamu.edu/mahapatra/teaching/ch3.pdf


"""
Our ROS node will create a Parabolic Trajectory Planner object.
"""



class ParabolicTrajectoryPlanner(object):

    def __init__(self):
        self.trajectory = []  # List of (x, y, z) tuples for the current locations

    def add_to_trajectory(self, current_location):
        self.trajectory.append(current_location)

    def predict(self, timesteps, trajectory):  # predict location of the object based on the trajectory so far after t timesteps.

        if len(trajectory) > 1:
            for i in range(1, len(trajectory)):

                xp, yp, zp = trajectory[i - 1]
                x, y, z = trajectory[i]

                delta_z = zp - z  # Distance changed in time step for the frame 
                delta_x = xp - x

                delta_h = sqrt(pow(delta_x, 2) + pow(delta_z, 2))  # Projected horizontal movement to be used for the final trajectory plan
                # This should be a constant change so our horizonal_final should be timesteps * delta_h 
                # (* some degradation factor to take into account the full path including earlier timesteps)


                """
                Then we find the final horizontal displacement after timesteps and breakdown this displacement into
                x and z components and adjust x and z in our coordinate frame accordingly.

                We can find y with a couple methods:
                1. Just plugging in the y coords with the equally displaced horizontals into numpy with degree 2 parabola and taking the h = timesteps, y value on the parabola
                2. Using physics equations and gravity acceleration decay to find the y position after timesteps
                3. Measuring the angle from the most recent point(s) and using that to get a current position vector... then im not sure what next.

                For all of these methods, must determine how to use all the x, y, z values and not just the most recent one.
                """
        # return predicted_location

return ParabolicTrajectoryPlanner(path)
