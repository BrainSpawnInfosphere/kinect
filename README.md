# ROS Stack: Kinect

**Original Authors:** Willow Garage

**Current Author:** Kevin Walchko

**License:** BSD

**Language:** C++

**Website:** http://github.com/walchko/kinect

This stack was resurrected from the old kinect tools which used libfreenect.

### Homebrew Dependencies
The required homebrew formulas for OSX can be installed using:

    cd kinect/homebrew
    brew update
    brew install -v ./pcl.rb
    brew uninstall libusb
    brew install -v ./libusb-freenect.rb
    brew install -v ./libfreenect.rb

Note that the USB library used here is *not* the standard USB library but a
patched version specifically used for the Kinect. Also, this library will install
over (replace) the standard USB library if you have installed it already. This is 
why it is uninstalled before installing libusb-freenect.

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


## Viewer Node: Kinect Camera Point Cloud Viewer

**Author:** Kevin Walchko

**License:** BSD

**Language:** C++

**Website:** http://github.com/walchko/kinect/pcl_view

A Glut window which converts the kinect depth image (uint16_t) into a pcl::PointCloud<PointXYZ> 
and displays it in 3D. The window allows the user to rotate and zoom the point cloud.

### Command Line

	rosrun viewer pcl_view -c /my/cloud -d /my/image

* c: cloud input
* d: depth image input

#### Example:

 	rosrun viewer pcl_view -c "/camera/cloud"

### To Do

* Clean-up code
* Handle coloring the point cloud better - it is messed up currently
* Incorperate the RGB image
* Allow easy incorperation of pcl filters
* More optimizations so it runs better
* Get the cloud size and center the camera on it


## Viewer Node: Kinect Camera Depth Viewer

**Author:** Kevin Walchko

**License:** BSD

**Language:** C++

**Website:** http://github.com/walchko/kinect/depth_view

Uses OpenCV to convert the uint16_t image into an image and displays it.

### Command Line

	rosrun veiwer depth_view /my/image

#### Example:

 	rosrun viewer depth_view /camera/image

### To Do

* Lots

## Other

### Virtualbox (or Other Linux Machine)

You can run the kinect_node on an OSX machine and then run other nodes in virtualbox.
Assuming two machines, OSX and VB (short for Virtualbox, but could also be another
real machine running Linux) set them up as follows:

#### OSX (Master)

* Before running any ROS nodes, do "export ROS_HOSTNAME=<IPADDR>" where you put in
the real IP address of your OSX machine.
* Launch roscore and kinect_node (and any other nodes you need)

#### VB

* Before running any ROS nodes, do "export ROS_MASTER_URI=http://<IPADDR>:11311" where 
<IPADDR> is the IP address of your OSX master.

If you have roscore and kinect_node running on OSX, you should be able to do:
"rosrun image_view image_view image:=/camera/rgb/image_raw" and see an image from
your kinect camera.