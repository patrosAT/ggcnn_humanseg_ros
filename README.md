# GGCNN ROS package with human parts filtering #

This is a ROS wrapper of the [ggcnn network](https://github.com/dougsm/ggcnn) that segments human body parts and sets their picking probability to 0. Based on inputs of the 1) depth camera and the ros packages 2) bodyparts_ros, 3) egohands_ros, and 4) darknet_ros, this ros package published a grasp map and the best picking point (location, gripper orientation, gripper width & quality). The grasp map is adapted to make sure the robot does not pick a human body part.

#### Input ####

* **Depth image:** [sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)
* **Bodyparts:** [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html)
* **Egohands:** [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html)
* **Yolo:** [sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)

#### Output ####

* **Best picking point:** [GraspPrediction](/msg/GraspPrediction.msg)
* **Heat Map:** [sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html)

## Getting Started ##

### Dependencies ###

The models have been tested with Python 2.7 and 3.6.
The transformations are specific to the [Franka Emika robot arm](https://frankaemika.github.io/)

#### Hardware ####

* Depth Camera
* GPU > 4000MiB
 
#### Python3 / pip3 ####
```
numpy
scipy
cv2
tensorflow
```
#### Ros ####
```
rospy
sensor_msgs
geometry_msgs
tf2_ros
tf2_geometry_msgs
cv_bridge
```
#### Ros 3rd party packages ###
* [Bodyparts_ros](https://github.com/patrosAT/bodyparts_ros.git)
* [Egohands_ros](https://github.com/patrosAT/egohands_ros.git)
* [Darknet_ros](https://github.com/leggedrobotics/darknet_ros)

**Note:** To enable real-time processing it might be necessary to distribute these packages across several computers. We recommend using [sensor_msgs/CompressedImage](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CompressedImage.html)s to keep the network usage on a reasonable level.

### Bilding ###

To maximize performance, use the 'release' build mode:
```
catkin_make -DCMAKE_BUILD_TYPE=Release
```

### Configuration ###

The initial setup can be changed by adapting the [ggcnn_humanseg.yaml](cfg/ggcnn_humanseg.yaml) file:

#### Camera info ####
* **info:** Camera topic on which the camera info is published.
* **camera_frame:** The camera frame.
* **robot_base_frame:** The robot base frame.
* **fov:** The camera' field of view.

#### Camera ####
* **image:** Rostopic the node is subcribing to (rgb image).
* **depth:** Rostopic the node is subcribing to (depth image).

#### Subscription ####
* **bodyparts:** Rostopic the node is subcribing to (bodyparts).
* **egohands:** Rostopic the node is subcribing to (egohands).
* **darknet:** Rostopic the node is subcribing to (yolo).

#### Interface ####
* **topic:** Rostopic the publisher node is publishing to *(please do not change)*.

#### Ggcnn ####
* **crop_size:** GGCNN specific parameter *(please do not change)*.
* **crop_offset:** GGCNN specific parameter *(please do not change)*.
* **out_size:** GGCNN specific parameter *(please do not change)*.

#### Robot ####
* **dist_ignore:** Field of range in meter. Objects behind this distance are inored
* **gripper_width:** Height of the gripper in meter (= distance between the camera and the gripper).

#### Visualization ####

The visualization mode published the original image with the background blacked out. Please be aware that turing on the visualization increases computing time and network utilization substantially.

* **topic:** Topic the node is publishing to.
* **activated:** Turn on/off visualization: *use keywords "on" or "off"*.

### Launch

Before launching the package, make sure that the camera and the 3rd party ros packages are up and running. 

The ros package contains a launch file:
* **Publisher:** The [publisher](launch/ggcnn_humanseg_publisher.launch) launch file starts a ros node that published a new mask every time a new depth image is published.

### Visualization

If the visualization is set 'True', a heat map [sensor_msgs/Image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html) is published to the corresponding topic. 

## Acknowledgments

The ROS node is powered by the ggcnn of [dougsm](https://github.com/dougsm). For more information, please refer to the original [paper](https://arxiv.org/abs/1804.05172) or the following [github repository](https://github.com/dougsm/ggcnn)

## License

* **Academic:** The project is licensed under the 3-clause BSD License.
* **Commercial:** Please contact the author.
