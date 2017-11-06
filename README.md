# Ball Tracker Vision for Kinect2 and Robot Arm
Ball Tracker Vision ROS Package for Kinect2 sensor to track a ball in 3D space. Apply Kalman Filter for ball trajectory planning. Then actuate Robot arm with Model Predictive Control to catch the ball.


### Dependencies
#### Hardware
kinect2 sensor

#### Software
ROS
OpenKinect/freenect2
iai_kinect/kinect2_bridge

### Run
(In separate terminal windows)

* roslaunch kinect2_bridge kinect2_bridge.launch
* rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 world kinect2_rgb_optical_frame 100
* rosrun ball_tracker ball_tracker.py

#### Check Successful Tracking
* rosrun rqt_tf_tree rqt_tf_tree
* rosrun rviz rviz
    * add tf (ball)
    * add pointcloud2 (kinect2)