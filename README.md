# Ball Tracker Vision for Kinect2 and Robot Arm
Ball Tracker Vision ROS Package for Kinect2 sensor to track a ball in 3D space. Apply Kalman Filter for ball trajectory planning. Then actuate Robot arm with Model Predictive Control to catch the ball.


### Dependencies
#### Hardware
* kinect2 sensor

#### Software
* ROS
* OpenKinect/freenect2
* iai_kinect/kinect2_bridge

---

### Run
(In separate terminal windows)

* roslaunch kinect2_bridge kinect2_bridge.launch
* rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 world kinect2_rgb_optical_frame 100
* rosrun ball_tracker ball_tracker.py
* rosrun ball_tracker ball_kalman_filter.py

#### Check Successful Tracking
* rosrun rqt_tf_tree rqt_tf_tree
* rosrun rviz rviz
    * add tf (ball)
    * add pointcloud2 (kinect2)

---

#### Features to note for Similar Projects:
* Read from Point Cloud 2 via pc2.read_points() function
* Use message_filters.TimeSynchronizer() to synch different times at same frequency via ROS message_filters.
* Broadcast TF and create TF listener
* Bitmasking and analyze subwindow contours via OpenCV
