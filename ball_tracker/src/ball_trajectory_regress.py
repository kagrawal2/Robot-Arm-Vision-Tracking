#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the dependencies as described in example_pub.py
# import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import fsolve
# from std_msgs.msg import Header
# from geometry_msgs.msg import Pose, Point, Quaternion, Accel, Vector3, AccelStamped
# from geometry_msgs.msg import PointStamped, PoseStamped
# from geometry_msgs.msg import TransformStamped
# import tf


class BallTrajectoryPlanner(object):

    def __init__(self):
        self.initialize_path_tracking = False
        self.seen_point_queue = None

    def reset_point_queue(self):
        self.seen_point_queue = None

    def add_seen_point(self, xyz):
        if self.seen_point_queue is None:
            self.seen_point_queue = np.array(xyz)
        else:
            self.seen_point_queue = np.append(self.seen_point_queue, xyz, axis=0)

    def lin_reg_xz(self):
        t = np.linspace(0, len(self.seen_point_queue), len(self.seen_point_queue))  # timesteps
        A = np.vstack([t, np.ones(len(t))]).T
        seen_xs = self.seen_point_queue[:,0]
        xm, xc = np.linalg.lstsq(A, seen_xs)[0]
        # print(xm, xc)

        seen_zs = self.seen_point_queue[:,2]
        zm, zc = np.linalg.lstsq(A, seen_zs)[0]
        # print(zm, zc)

        return xm, xc, zm, zc

    def lin_reg_y(self):
        t = np.linspace(0.0, len(self.seen_point_queue), len(self.seen_point_queue))  # timesteps
        A = np.vstack([np.power(t, 2), t, np.ones(len(t))]).T
        seen_ys = self.seen_point_queue[:,1]
        a, m, c = np.linalg.lstsq(A, seen_ys)[0]
        # print(a, m, c)
        return a, m, c


    def solve_sphere_intersection(self, center, radius, init_t):
        x2, x1, z2, z1 = self.lin_reg_xz()
        y3, y2, y1 = self.lin_reg_y()
        c1, c2, c3 = center
        r = radius

        func = lambda t: (x1-c1 + x2*t)**2 + (y1-c2 + y2*t + y3*(t**2))**2 + (z1-c3 + z2*t)**2 - r**2  # solves for this function set = 0
        t_sol = fsolve(func, init_t)
        # print(t_sol)

        x_sol = x2*t_sol + x1
        y_sol = (t_sol**2)*y3 + y2*t + y1
        z_sol = z2*t_sol + z1
        return x_sol, y_sol, z_sol



t = np.linspace(0, 50, 50)  # timesteps
g = -9.81  # m/s^2 acceleration due to gravity (only used in the intersection time solver)
# Parameterized by t (time in seconds)
xs = 0.3 * t #+ 4.0 
ys = g*pow(t, 2) + 4.0*t + 5.0 
zs = 0.2 * t #+ 2.0 


center = [0, 0, 0]
radius = 48

# Create a sphere
r = radius
pi = np.pi
cos = np.cos
sin = np.sin
phi, theta = np.mgrid[0.0:pi:100j, 0.0:2.0*pi:100j]
x = r*sin(phi)*cos(theta) + center[0]
y = r*sin(phi)*sin(theta) + center[1]
z = r*cos(phi) + center[2]




btp = BallTrajectoryPlanner()
btp.add_seen_point([[xs[0], ys[0], zs[0]]])
pred_xs, pred_ys, pred_zs = [0], [0], [0]

for i in range(1, len(t)):
    btp.add_seen_point([[xs[i], ys[i], zs[i]]])
    # if i < 10:
        # xm, xc, zm, zc = btp.lin_reg_xz()
        # ya, ym, yc = btp.lin_reg_y()

    x_sol, y_sol, z_sol = btp.solve_sphere_intersection(center, radius, 45)
    pred_xs.extend(x_sol)
    pred_ys.extend([y_sol[0]])
    pred_zs.extend(z_sol)



from mpl_toolkits.mplot3d import Axes3D

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot_surface(x, y, z, rstride=1, cstride=1, color='c', alpha=0.6, linewidth=0)
ax.plot(xs, zs, ys, label='parametric curve')
ax.legend()
# plt.show()
# from mpl_toolkits.mplot3d import Axes3D

# fig = plt.figure()
# ax1 = fig.add_subplot(111, projection='3d')
ax.plot(pred_xs, pred_zs, zs=pred_ys, color='r', label='parametric curve')
ax.legend()
plt.show()


# def listener():
#     from time import sleep
#     sleep(1)

#     rospy.init_node('ball_trajectory_planner', anonymous=True)

#     tf_listener = tf.TransformListener()
#     rate = rospy.Rate(30.0)
#     sleep(3)
#     frame_id = "kinect2_ir_optical_frame"
#     # br = tf.TransformBroadcaster()
#     # br.sendTransform((0, 0, 0),   tf.transformations.quaternion_from_euler(0, 0, 0),
#     #  rospy.Time(), frame_id, 'kinect_link')

#     base_trans, base_rot = tf_listener.lookupTransform('kinect_link','base_link', rospy.Time())
#     pub = rospy.Publisher('final_pose', PoseStamped, queue_size=10)
#     btp = BallTrajectoryPlanner()

#     while not rospy.is_shutdown():
#         try:
#             tf_time = tf_listener.getLatestCommonTime(frame_id, "ball")
#             ball_trans, ball_rot = tf_listener.lookupTransform(frame_id, "ball", rospy.Time())
#         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             continue
#         btp.predict_location(ball_trans, tf_time, frame_id)
#         rate.sleep()  # Check if this is necessary

#     #Wait for messages to arrive on the subscribed topics, and exit the node
#     #when it is killed with Ctrl+C
#     # rospy.spin()


#     #Python's syntax for a main() method
# if __name__ == '__main__':
#     listener()
