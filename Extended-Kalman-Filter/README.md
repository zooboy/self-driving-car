# **Extended-Kalman-Filter** 




[//]: # (Image References)

[image1]: /RMSE.png "RMSE"




#### 1.Understand Extended Kalman Filter 
The Extended Kalman Filter is similar to classic Kalman Filter,except using approximate linear equation to replaced the non-linear one, as  F and H matrix.when calculating P′.
the H matrix in the Kalman filter will be replaced by the Jacobian matrix Hj
​​when calculating S, K, and P.to calculate x​′, the prediction update function, f, is used instead of the F matrix.to calculate y, the h function is used instead of the H matrix.And for this project, we do not need to replace  F by f,so the predict part is just the same  to lidar and radar sensor. 


#### 2.implement coding work
The two types of sensor（lidar and radar） use the same predict function,so I just use the kalman filter equation to do it, you can see the code and the notes in kalman_filter.cpp.for lidar update ,it is same as the classic kalman filter,because its state transition function and measurement function are linear equation.for radar udpate ,I need to convert the predict value (Px,Py,Vx,Vy) to ρ φ and ρ.dot, it is the h function,then I use the pred vector to calculate the deviation, 
y = z - z（pred）. After that , I normalize the y(1),it is the difference between  
φ predicted and φ measurement,it need to be scaled in range [-pi , pi].Then I used the Hj to caculate the kalman gain, the Hj have been calculated in Tools::CalculateJacobian() funtion,finally, I update the X and P estimate.I have noted the code in the .cpp file ,you can check it.


In the tools.cpp,I defined the RMSE and Jacobian Matrix caculate function.

In the FusionEKF part,I initialize the measurement covariance matrix and the P matrix.Then for first frame,I initialize the state and get the measurement values.Especially for radar,I convert the measurement values to cartesian from polar. Then I define the update equation for F and Q matrix.After that, I define the process workflow based on the type of sensor.


#### 3.Test and debug
The code need to compile in ubuntu environment, and the debug runtime error information is hard to point where was wrong exactly. So I add some print statement to point the error.Finally I get the RMSE [0.0974,0.0854,0.4455,0.4451].
 ![alt text][image1]

comment see in .cpp file.

------------

### Dependencies

This project involves Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO. Please see [this concept in the classroom](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77) for the required version and installation scripts.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ExtendedKF

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

Note that the programs that need to be written to accomplish the project are src/FusionEKF.cpp, src/FusionEKF.h, kalman_filter.cpp, kalman_filter.h, tools.cpp, and tools.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.


INPUT: values provided by the simulator to the c++ program

["sensor_measurement"] => the measurement that the simulator observed (either lidar or radar)


OUTPUT: values provided by the c++ program to the simulator

["estimate_x"] <= kalman filter estimated position x
["estimate_y"] <= kalman filter estimated position y
["rmse_x"]
["rmse_y"]
["rmse_vx"]
["rmse_vy"]



#### Other Important Dependencies

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

#### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF `



-------------------------------

**email**:*zooboy@vip.sina.com*
 

