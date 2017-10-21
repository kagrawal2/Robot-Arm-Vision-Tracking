from ball_tracking import MaskBallTracker

tracker = MaskBallTracker()

path = tracker.get_path()  # Would normally be path so far.


class ParabolicTrajectoryPlanner(object):

    def __init__(self):
        pass

    def predict(self, timesteps, trajectory):  # predict location of the object based on the trajectory so far after t timesteps.

        if len(trajectory) > 1:
            for i in range(1, len(trajectory)):

                xp, yp, zp = trajectory[i - 1]
                x, y, z = trajectory[i]

                delta_z = z - zp  # Distance changed in time step for the frame 



ParabolicTrajectoryPlanner(path)
