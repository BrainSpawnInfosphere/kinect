# ROS Stack: Kinect

**Original Authors:** Willow Garage

**Current Author:** Kevin Walchko

**License:** BSD

**Language:** C++

**Website:** http://github.com/walchko/kinect

This stack was resurrected from the old kinect tools which used libfreenect. 

## ROS Node: Kinect Camera Node

**Current Author:** Kevin Walchko

**License:** BSD

**Language:** C++

**Website:** http://github.com/walchko/kinect/kinect_camera

### Command Line

	rosrun kinect_camera kinect_node
	
#### Example:

    rosrun kinect_camera kinect_node

#### Published Topics: 
**RGB Image:** 

"/camera/rgb/image_raw" Raw RGB image from camera

**Depth Image:** 

"/camera/depth/image_raw" Raw image from depth camera (encoding is mono16)

"/camera/depth/camera_info" Camera information

**Accelerometers:**

"/camera/imu" The readings from the accelerometers in the connect

### To Do

* Further clean-up and bring code up to Fuerte API standards
* Add motor control
* Add options for LED control
* Add microphone capability
* Add debugging capability
* Figure out what to do with the included freenect package


## ROS Node: Kinect Camera Point Cloud Viewer

**Author:** Kevin Walchko

**License:** BSD

**Language:** C++

**Website:** http://github.com/walchko/kinect/pcl

A Glut window which converts the kinect depth image (uint_16) into a pcl::PointCloud<PointXYZ> 
and displays it in 3D. The window allows the user to rotate and zoom the point cloud.

### Command Line

	rosrun pcl pcl

#### Example:

 	rosrun pcl pcl

### To Do

* Clean-up code
* Handle coloring the point cloud better
* Incorperate the RGB image
* Allow easy incorperation of pcl filters
* More optimizations so it runs better


