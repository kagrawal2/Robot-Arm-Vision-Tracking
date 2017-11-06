#!/usr/bin/env python

# import the necessary packages
import rospy
import message_filters

from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
import tf

from collections import deque
import numpy as np
import argparse
import cv2
import matplotlib.pyplot as plt
from colorsys import rgb_to_hsv


class MaskBallTracker(object):

    def __init__(self):
        ### Initialization Parameters for color range for the ball 
        # (HSV)
        self.GREEN_MIN = (45, 47, 47)
        self.GREEN_MAX = (90, 255, 255)

        self.PINK_MIN = (140, 98, 85)
        self.PINK_MAX = (165, 255, 255)
        # (HSV)
        self.YELLOW_MIN = (8, 50, 50)
        self.YELLOW_MAX = (45, 255, 255)
        # HSV for Tennis ball
        self.TENNIS_MIN = (30, 255, 135)
        self.TENNIS_MAX = (40, 255, 185)

        self.start_path_tracking = True
        self.cv_bridge = CvBridge()


    def track_ball_rgb_uv(self, image):

        # print("Got image")
        frame = self.cv_bridge.imgmsg_to_cv2(image, "bgr8")
        # rospy.loginfo("Converted image to CV2")
        # self.show_cv2_image(frame)

        # blur cv2 frame, and convert it to the HSV
        # color space
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # self.show_cv2_image(hsv)

        # construct a mask for the desired color, then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        green_mask = cv2.inRange(hsv, self.GREEN_MIN, self.GREEN_MAX)
        green_mask = cv2.erode(green_mask, None, iterations=1)
        green_mask = cv2.dilate(green_mask, None, iterations=1)

        pink_mask = cv2.inRange(hsv, self.PINK_MIN, self.PINK_MAX)
        pink_mask = cv2.erode(pink_mask, None, iterations=1)
        pink_mask = cv2.dilate(pink_mask, None, iterations=1)

        # self.show_cv2_image(mask)
        # self.show_cv2_image(pink_mask)
        mask = cv2.bitwise_or(green_mask, pink_mask)  # Use to combine the two images masks for the given ball
        # mask = green_mask
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # only proceed if the radius meets a minimum size
            if radius > 9:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                # cv2.circle(frame, (int(x), int(y)), int(radius),
                #     (0, 255, 255), 2)
                # cv2.circle(frame, center, 5, (0, 0, 255), -1)

                # if not self.start_path_tracking:
                #     if raw_input("Initialize Path Tracking?: (y/n)") == "y":
                #         self.start_path_tracking = True

                if self.start_path_tracking:
                    u, v = center[0], center[1]

                    # self.show_cv2_image(frame)
                    return u, v
                  
        # cleanup the camera and close any open windows
        cv2.destroyAllWindows()

        return None, None

    def track_ball_cam_coord_point_cloud(self, rgb_img, point_cloud):
        # Convert from rgb+rectified_image u,v to synchronized ir_point cloud X, Y, Z in the camera frame

        u, v = self.track_ball_rgb_uv(rgb_img)
        # print(u, v)

        # If the ball (u, v) pixel is found
        if u is not None:
            # Read point cloud for Camera Frame X, Y, Z for the pixel (u, v) found
            cam_x, cam_y, cam_z = self.get_cam_coord_point_cloud(u, v, point_cloud)
            if cam_x:
                # Broadcast X, Y, Z, timestamp as a TF
                self.tf_broadcast(cam_x, cam_y, cam_z, rgb_img.header.stamp)

    def get_cam_coord_point_cloud(self, u, v, point_cloud):
        # Read point cloud for Camera Frame X, Y, Z for the pixel (u, v) found
        data_out = pc2.read_points(point_cloud, field_names=("x","y","z"), skip_nans=True, uvs=[[u, v]])
        if data_out:
            try:
                cam_x, cam_y, cam_z = next(data_out)
            except Exception:
                return None, None, None
            return cam_x, cam_y, cam_z

    @staticmethod
    def publish_ball(x, y, z, ts):
        # May be used to just publish, X, Y, Z, timestamp geometry msg to a topic
        pass

    @staticmethod
    def tf_broadcast(x, y, z, ts):
        br = tf.TransformBroadcaster()
        br.sendTransform((x, y, z), tf.transformations.quaternion_from_euler(0, 0, 0), ts, "ball", "world")

    @staticmethod
    def show_cv2_image(img):
        while True:
            cv2.imshow("Image window", img)
            if cv2.waitKey(33) == ord("a"):
                break


# Main function used to track ball
def track_ball():

    BallTracker = MaskBallTracker()
    rospy.init_node("ball_tracker", anonymous= True)

    ### Time Synchronized Messages ###
    rgb_image_sub = message_filters.Subscriber("/kinect2/qhd/image_color_rect", Image)
    point_cloud_sub = message_filters.Subscriber("/kinect2/qhd/points", PointCloud2)
    synch_sub = message_filters.TimeSynchronizer([rgb_image_sub, point_cloud_sub], 8)
    synch_sub.registerCallback(BallTracker.track_ball_cam_coord_point_cloud)

    rospy.spin()

if __name__ == "__main__":
    track_ball()