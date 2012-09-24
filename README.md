# ROS Stack: Kinect

**Original Authors:** Willow Garage

**Current Author:** Kevin Walchko

**License:** BSD

**Language:** C++

**Website:** http://github.com/walchko/kinect

This stack was resurrected from the old kinect tools which used libfreenect. I have had
various difficulties getting OpenNI libraries working on OSX.

### Homebrew Dependencies
The required homebrew formulas for OSX can be installed using:

    cd kinect/homebrew
    brew update
    brew install -v ./pcl.rb
    brew install -v libusb
    brew install -v ./libfreenect.rb

Note that the USB library used here *is* the standard USB library and the patches 
for the Kinect are included in version 1.09. Also, this library will install
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

## PCL Tools

**Author:** Kevin Walchko

**License:** BSD

**Language:** C++

A couple of key tools to make like with ROS and PCL better on OSX:

### Point Cloud Maker

A node that takes in a kinect 16b depth image and converts it to a point cloud. 
Currently it color codes the points according to depth only.

#### To Do

* Color code each point according to the kinect RGB image.

### Point Cloud Viewer

Actually this takes a kinect 16b depth image and using the CloudMaker node, convert
it into a point cloud. Then it displays it in an OpenGL 3D widow. This window is 
for visualization only, so a user modified CloudMaker node could do a lot of things
to the point cloud before it gets displayed. The main advantage of this is there is 
no transmission of the point cloud over the network and there is no need to use
the overly complicated nodelet scheme.  

kinect -> depth_image -> cloud_maker -> viewer


# Other Notes (Need to move somewhere else)

## Remote Master

You can run the kinect_node on an OSX machine and then run other nodes in virtualbox.
Assuming two machines, OSX and VB (short for Virtualbox, but could also be another
real machine running Linux) set them up as follows:

### OSX (Master)

* Before running any ROS nodes, do "export ROS_HOSTNAME=<IPADDR>" where you put in
the real IP address of your OSX machine.
* Launch roscore and kinect_node (and any other nodes you need)

### VB (Slave)

* Before running any ROS nodes, do "export ROS_MASTER_URI=http://<IPADDR>:11311" where 
<IPADDR> is the IP address of your the master computer.

If you have roscore and kinect_node running on OSX, you should be able to do:
"rosrun image_view image_view image:=/camera/rgb/image_raw" and see an image from
your kinect camera.

# Virtualbox

Not everything runs on OSX right now. There are some difficulties with OpenNI and some of 
the graphical stuff that makes it necessary to be able to run Linux. However, as a 
staunch Mac enthusiast, I prefer to run Linux in a virtual machine (vm) rather than get a
separate computer to run it on.

My typical setup is a Mac as the host computer and Linux in a virtual machine. This allows
me to run some of the code (OpenNI stuff, SLAM, etc) in the vm and have my Mac connected
to the hardware. **Note** that you cannot have a vm get access to a Kinect or really any
useful hardware. 

[kinect] ---> [mac] <---> [vm]
                ^
                |
[robot] <-------+

## Networking

There are several types of networks you can setup for your vm's:

* **NAT** - each vm is on its own private network using a virtual network device. The vm can 
ping the host, but the host cannot ping the vm. This isn't that useful.
* **Bridging** - the vm binds to a real network device (wired or wireless) and becomes a
computer on the network. This is useful when you want to run a vm on one computer and have
another computer (or vm on another computer) talk with it.
* **Host Only** - all vm's and the host use a virtual network device and can talk to each 
other, but the vm's are not seen on the network. Make sure to go into the Preferences
and create a host-only device. This determines the DHCP parameters that will be used.

Also, install the [avahi](avahi.org) tools installed which will implement the Zeroconfig
(Bonjour) protocol so you can communicate easier with your vm's. To find what is on
your network, type:

    avahi-discover

This should list all of the vm's and the host computer. Additionally, you should be able
to get results with:

    ping <HOSTNAME>.local

where <HOSTNAME> is the name of the host computer.

## SSH Server

You will need to setup your virtual machine (vm) with ssh client and ssh server so you 
can remotely connect to it. The following are examples of configuration directives you 
may change in the /etc/ssh/sshd_config file:

To set your OpenSSH to listen on TCP port 2222 instead of the default TCP port 22, change 
the Port directive as such:

    Port 2222

To have sshd allow public key-based login credentials, simply add or modify the line:

    PubkeyAuthentication yes

To make your OpenSSH server display the contents of the /etc/issue.net file as a 
pre-login banner, simply add or modify the line:

    Banner /etc/issue.net

After making changes to the /etc/ssh/sshd_config file, save the file, and restart the 
sshd server application to effect the changes using the following command at a terminal 
prompt:

    sudo /etc/init.d/ssh restart

or

    sudo restart ssh

Ubuntu allows you to use stop, start, or restart ssh instead of calling the init.d script 
directly.

### SSH Keys
SSH keys allow authentication between two hosts without the need of a password. SSH key 
authentication uses two keys a private key and a public key.

To generate the keys, from a terminal prompt enter:

    ssh-keygen -t dsa

This will generate the keys using a DSA authentication identity of the user. During the 
process you will be prompted for a password. Simply hit Enter when prompted to create the 
key with an empty pass phrase.

By default the public key is saved in the file ~/.ssh/id_dsa.pub, while ~/.ssh/id_dsa is 
the private key. Now copy the id_dsa.pub file to the remote host and append it to 
~/.ssh/authorized_keys by entering:

    ssh-copy-id username@remotehost

Finally, double check the permissions on the authorized_keys file, only the authenticated 
user should have read and write permissions. If the permissions are not correct change 
them by:

    chmod 600 .ssh/authorized_keys

You should now be able to SSH to the host without being prompted for a password.

## Running Headless VMs

You can launch Virtualbox headless on OSX using the following command:

    /Applications/VirtualBox.app/Contents/MacOS/VBoxHeadless --startvm <VM_NAME>

where <VM_NAME> is the name of the vm you want to launch. **Note** that
in OSX, the this will segfault if you have enabled 3D Acceleration in the Display 
settings.

## Connecting

Please replace the <HOSTNAME> and <USERNAME> with the proper host and user names.

1. launch the vm as described above
2. double check it is up and running: ping <HOSTNAME>.local
3. login: ssh <USERNAME>@<HOSTNAME>.local
4. Setup ROS environments on host and vm's properly. Run ros nodes ...
5. When done ... sudo poweroff