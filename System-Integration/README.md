#**System Integration Project** 




##

  **Email :  *zooboy@vip.sina.com***  
  

##
[//]: # (Image References)

[image1]: ./arc.jpg "arg"

### Introduction
I implement this project myself alone,The ROS system architecture of this project like below.
![alt text][image1]

According to the requirements , I implement the function of these nodes: Waypoint updater,DBW and Traffic light detection. Now, the system can smoothly follow the waypoints at a setting top speed and stop at traffic lights when needed in simulator mode . It can detect both the simulator and the real world red lights by camara images.  

Since the walkthrouth of classroom had provided some basic code of these moudules, So here I mainly introduce what I've done besides the walkthrough code.

#### Waypoint Updater Node

The walkthrough code had contained finding the closest waypoint function which used a KDTree method.And the waypoint follower node had provided acceleration function, and these two parts perform quite well. Only one part I think need to improve is the deceleration function used for car red lights stop, the velocity decreased too sharp and sometimes would cause a high jerk. I update the velocity deceleration formula with a linear decrease which relate with the distance to the stop line. I commented the code in waypoint_updater.py in line 133 to 141. Maybe there were some better way to deal with it ,like Minimize Jerk or some sigmoid transformer funtion,  I 'll try some other methods in the future for this part. 


#### DBW Node

As mentioned in project walkthrough, the car has a little shifting around the reference trajectory , and there are 2 kinds of solution for  this issue, one way is to modify the waypoints follower .cpp code to enforce the follower node ajust car's  pose at each waypoint . The other way is to add some damping to the steer. I choosed the latter option . I add a pid controller to ajust the car's steer in order to decrease shifting. So, firstly calculate the CTE value(code and comment in line 118 to 149 in dbw_node.py), then build the pid controller and finetune  pid parameters. 



####Traffic Light Detection Node
For this node, I used two methods to detect the red lights, for simulator part,the color of images of camara captured is pure ,clear and uniformity, so  I used an open-cv detection solution (Honestly, I learned this method form 'slack' that some students had experienced),the main track of this solution is to filter the red color out of the image, and check the shape of the red part(the shape of red light like a circle).the code and comment in line 73 to 116 in tf_classfier.py, the test effect of this method in file: 
```
light_classification/simulator_traffic_lights_detect.ipynb.
```
The performance of this method is good, it can recognize the red lights precisely and quickly , but it only can be used in simulator mode, for realworld images detection, the color of red light is fuzzy,so I used a CNN classifier to deal with it.The train file : 
```
light_classification/traffic_lights_detect_training.ipynb 
```
which using a keras package to build the nn.The dataset comes from two source: ROSbag file from Carla and part of LaRA Traffic Lights Recognition Dataset, I have test the classifier in site mode, it performs not bad.By the way, in process code of this part, I only consider 2 detection results : red light or not ,it is quite simple and easy to act in waypoint updater node.(I think it should be more reasonable if I take some action in yellow light )

There is some points to notice for this part :

I add a config const 'sim' in both ```sim_traffic_light_config.yaml``` and ```site_traffic_light_config.yaml```
to mark whether if the simulator mode red light detection works. In tl_detector node ,I'll check this parameter to choose different processing procedure.

I add a debug switch variable called ```self.site_test_mode``` in tl_detector.py in order to check the detection effect in site mode with the training rosbag file. Now, the variable is turn off, you can turn it on if need.

The keras package seems have a compatibility problem when loading model file ```*.h5```  if trained a model in local machine with python3.x and used a ```lambda``` function for images batch processing inner your keras training model, to fix the problem, just remove the  ```lambda```function in your code. I think this problem need to remind  the other students who uses a keras package to train their model. 

================================================================================================================

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
