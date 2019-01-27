# RatSLAM on a Humanoid

<img src='https://wiki.qut.edu.au/download/attachments/104094381/logo_sml.jpg?version=1&modificationDate=1338441816000'>

<b>RatSLAM</b> is a bio-inspired simultaneous localisation and mapping (SLAM) system. Based on continous attractor network dynamics, RatSLAM is capable of mapping by closing loops to correct odometry error.<br>
<br>
The original RatSLAM algorithm was designed and implemented on Pioneer robots by Michael Milford and Gordon Wyeth (see <a href='http://eprints.qut.edu.au/37593/1/c37593.pdf'>RatSLAM: a hippocampal model for simultaneous localization and mapping</a>).<br>
<br>
There is an openRatSLAM paper available in <a href='http://www.springerlink.com/openurl.asp?genre=article&id=doi:10.1007/s10514-012-9317-9'>Autonomous Robots</a>. This paper describes how openRatSLAM works in technical detail.  If you use the code we would appreciate cites please.<br>
<br>
This fork contains the code to a university project.
The aim of this project is to run the ROS-based version of RatSLAM on a Humanoid robot called [NAO](https://www.softbankrobotics.com/emea/en/nao) and implement the idea of Stefan Müller, Cornelius Weber und Stefan Wernter [(see RatSLAM on Humanoids - A Bio-Inspired SLAM Model Adapted to a Humanoid Robot](https://pdfs.semanticscholar.org/f660/8cfde283e07c8e634f9493df654356aa69a8.pdf).
<br>
In addition to the original code, the repository includes:
	- the implementation of the algorithm developed from [RatSLAM on Humanoids](https://pdfs.semanticscholar.org/f660/8cfde283e07c8e634f9493df654356aa69a8.pdf)
	- Some simple [choregraphe](http://doc.aldebaran.com/1-14/software/choregraphe/choregraphe_overview.html) scripts to move the NAO.
	- Some small bug fixes and adjustments to the NAO robot
	- example bagfiles (soon)
	- some videos for demonstration (soon)
<br>
More details to the original OpenRatSLAM implementation can be found [here](https://github.com/davidmball/ratslam/blob/wiki/RatSLAMROS.md).
The original OpenRatSLAM code was released under the GNU GPL V3.

<br>

Additionally, this fork contains:

## Installation
The tests were performed on Ubuntu 16.04 and [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) (version 1.12.13).

### Dependencies
ROS packages: opencv2 and topological_nav_msgs (if not installed by default), C++11

irrlicht:  ```sudo apt-get install libirrlicht-dev```

OpenCV (2!):  ```sudo apt-get install libopencv-dev``` <br>
If an compilation error occurs due to the OpenCV Version, make sure that OpenCV 2 is installed. [This](https://gist.github.com/arthurbeggs/06df46af94af7f261513934e56103b30) might solve the problem.<br>
If there is still an error, uncomment "{OpenCV_LIBRARIES}" in line 70 and 100 in "catkin_ws/src/ratslam_ros/CMakeLists.txt".

This should be sufficient for using RatSLAM with bagfiles, for experiments on the real NAO install the [NAO ROS Package](http://wiki.ros.org/nao).<br>
Packages can be installed using ```sudo apt-get install ros-kinetic-nao-robot```.<br>
Install specifig packages via ```sudo apt-get install ros-kinetic-nao*```.


For a virtual stand-alone version of NAO, install:<br>
[NAOqi C++ SDK](http://doc.aldebaran.com/2-1/dev/python/install_guide.html),<br>
[NAOqi Python SDK](http://doc.aldebaran.com/2-1/dev/cpp/install_guide.html)<br>
An aldebaran license is necessary to install Naoqi.<br>
Installation instructions are [available](http://wiki.ros.org/nao/Tutorials/Installation).<br>
A precompiled version of NAOqi is included in webots but did not work in my experiments!

Note: NAOqi works with Python 2.7.


### Build instructions
The implementation is built with "catkin":<br>
Execute the following commands
```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd..
catkin_make
source devel/setup.bash
```
Now clone the repository to "~/catkin_ws/src", replace the CMakeLists.txt and build it again: 

```
cd ~/catkin_ws
catkin_make
```

### Running the implementation

Open a terminal and start ROS via ```roscore```
<br>
Open a new terminal in the "~/catkin_ws/" directory and start RatSLAM via
```
roslaunch ratslam_ros nao.launch
```
If it is neccessary to build the code, use the command ```catkin_make``` before starting ratslam_ros. If ROS does not find ratslam_ros, you might have to execute ```source devel/setup.bash``` (or add this command directly to "~/.bashrc").
Play a bagfile via
```
rosbag play <file_name.bag>
```
Connect to a NAO via:
```
roslaunch nao_bringup_py nao_full_py.launch nao_ip:=<robot Ip> nao_port:=<port>
```
The port should be 9559.<br>
NAOqi can create a stand-alone virtual robot when it's started via "./naoqi" in the SDK Folder.

### Graphical interface
After running a bagfile or connecting the implementation to a real robot, three windows should appear:

- The Local View with the current image, the current template and the current image converted to a template below.
- The Pose Cell Network
- The Experience Map


## General notes

- If the lower part of the Local View interface is black, the path to the image topic might be wrong.
- If the NAO figure does not move in the Experience Map interface, the path to the odometry topic might be wrong.
- My changes in the code are marked with "CHR", or just compare the files to the original openRatSLAM implementation.

More notes to [OpenRatSLAM](https://github.com/davidmball/ratslam/blob/wiki/RatSLAMROS.md).


## Implementation details
The experimental setup:
<img src='https://github.com/christianlandgraf/ratslam/edit/ratslam_ros/figures/openratslam.png' alt='OpenRatSLAM'>

The original RatSLAM was developed for and tested only on wheeled robots.
Therefore, the following adjustments are necessary:
	- A new ROS launch file and a new ROS configuration file was created
	- The Visual Odometry data is not used because the NAO provides odometry data
	- "image_transport" is set to "raw"
	- the ROS nodes subscribe to the topics of the NAO ROS package, which publishes the image and odometry data (depending on the NAO version, this needs to be adjusted)
	- the calculation of the current velocity is adjusted. OpenRatSLAM uses the current translational and rotational speed, but the NAO ROS package provides the current position and rotation (as Quaternion) in space. The Quaternion conversion is done by the "irrlicht" library (Note: The tf C++-library does not work!!).
	- the multiple pose hypotheses algorithm from [Müller et al.](https://pdfs.semanticscholar.org/f660/8cfde283e07c8e634f9493df654356aa69a8.pdf) is implemented

The following sequence diagram illustrates the multiple pose hypotheses algorithm:
<img src='https://github.com/christianlandgraf/ratslam/edit/ratslam_ros/figures/sequencediagram.png' alt='Sequence Diagram'>


## Results

