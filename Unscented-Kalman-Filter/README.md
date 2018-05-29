#**Unscented-Kalman-Filter** 




[//]: # (Image References)

[image1]: /RMSEresult.png "RMSE"
[image2]: /NIS.png "NIS"



####1. Understand the framework  
the framework mainly has 3 parts : the main.cpp, to read the sensor data and drive the simulator work; the ukf.cpp , to accomplish the unscented Kalman Filter function; the tools.cpp, to test the performance of the filter by RMSE function. The Kalman Filter need to be accomplished in file ukf.cpp.To make the UKF work, the project need to complete these functions :1.ProcessMeasurement(),to initialize the state and describe how the filter work;2.Prediction(),to execute the prediction process of UKF;3.UpdateRadar(),to update the tracked object motion state based radar sensor data;4.UpdateLidar(),to update the tracked object motion state based lidar sensor data.

####2.code the function ProcessMeasurement()
In the function ProcessMeasurement(), I mainly divide the code to 3 parts:


1.initializing the variables of state,like x_,P_,n_x,n_aug,lambda, weights_,the shape of Xsig_pred_ and so on.I initialize the P_ as an identity matrix,n_x = 5,n_aug = 7 ,lambda = 3- n_aug.the weights value set as the course recommended.Besides this ,I add 4 variables in ukf.h file,in order to record the NIS value and the chi-square ratio,these 4 variables are initialized in ProcessMeasurement() initialize step.I initialize the X state by judging the sensor type.If the data come from radar,that means the first 3 values the radar feedback are pho,phi,pho-dot,and we can map these value to CTRV space like this:
		
py = sin(phi)*rho;

px = cos(phi)*rho;

v = rho*rho_dot;

yaw = phi(Radar)

and I think I can not know the yaw change rate( yaw-dot) for the first I receive the data from radar,so I set the yaw-dot as 0 for initializing value.So, for radar mode , the X state initializing value was :x_ = [px,py,v,yaw,0];

For lidar,for the first time we receive the data, we only know the position information,so I initialize the X state to x_= [px,py,0,0,0].

2.Once finishing the initializing step, the UKF execute prediction step,so I call the Prediction() function.

3.After prediction,the UKF execute the update process,I call the UpdateLidar() function or UpdateRadar() function base on which kind of sensor data the UKF received.  


####3.code the function Prediction()
I think there are mainly 3 steps to process in Prediction() funtion:
firstly, generate the augmented sigma points.
secondly, create predict sigma points 
thirdly, inverse the process of sigma points generating, to get the predicted x and P,the code in this function is the same as the quiz in lesson.I add some notes to the code,you can check them in ukf.cpp file.

####4.code for the function UpdateLidar() 
In lidar mode, the lidar only measured two values, the px and py,the 'H ' function is a matrix as: 

	1,0,0,0,0
	0,1,0,0,0

the measurement process is a linear equation,that means we can calculte the update value following the classic Kalman Filter update method.  

at the end of this function,I calculate the NIS value and statics the ratio of NIS value that lower than 2-dimensions chi-square x-0.95.I print these information for every iteration.  


####5.code for the function UpdateRadar() 

the radar measurement is not a linear process, so ,it need to transform the predicted sigma points to radar measurment space,then calculate the final predict vector Z-pred  and measurement covariance matrix S based the weights of each sigma points of Z predicted matrix.I note the code in each step. In the radar part, it is very important to normalize the angle to [-pi,pi],otherwise,it would lead wrong result.at the end of this function I calculte the NIS value and statics the ratio that NIS lower than 3-dimensions chi-square x-0.95.

####6.finetune the motion noise.
I print the lidar and radar NIS and the ratio lower than standard NIS value for each iteration.I observe these information to finetune the std_a_ and std_yawdd_, according to 3-d x2-0.95 standard value 7.815 and 2-d x2-0.95 standard value 5.991,if the statics result much higher than them,it would mean the noise I may be overestimated,on contrary,it would mean the noise I may be underestimated.Finaly I set the std_a_ = 1.5,and the std_yawdd_ =0.5, in this case ,the radar NIS ratio as 0.956,the lidar NIS ratio as 0.968.I think these values close to 0.95, and the sample points was not enough, so I think the setting value should be believable. On the other hand, consider a moving bike,the acceleration noise as 1.5m per second square,and the yaw change rate noise as 0.5( approximate 30 degree) , I think that make sense.  

#### the RMSE result here:
the RMSE is 

X:0.0691, 

Y:0.0826,

VX:0.3372,

VY:0.2195

The result meet this project RMSE requirment[0.09,0.1,0.4,0.3]

Besides this,the result is better than EKF project I've accomplished [0.0974,0.0854,0.4455,0.4451] 

 ![alt text][image1]

#### the NIS value and ratio I've printed out like this:

 ![alt text][image2]




#### for more details, check the source code.


----------------
### Dependencies

This project involves Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. 
Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF
### Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF` Previous versions use i/o from text files.  The current state uses i/o
from the simulator.
 
------------------
**email**:*zooboy@vip.sina.com*
