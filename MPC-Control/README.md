#**Model Predictive Control** 




[//]: # (Image References)

[image1]: ./formula1.png "formula"
[image2]: ./formula2.png "formula2"

### The project information and code running dependencies link : [project info](https://github.com/udacity/CarND-MPC-Project) 

#### 1.Understand the MPC Model
MPC is a good model that used in selfdriving car's motion state control.The main idea of MPC is that: finding a list of next continuous states by minimize the total cost which discribing the difference between the current state and the target state , and using the first few earlier actuators which have been predicted in that sequence as system input, and repeating that process.By MPC model, we can transform the motion state solving  to minimize cost optimization issue. 

In this project, we use IPOPT as optimizer, and CPPAD as automatic derivation calculator,by derivation method ,we can find local optimal solution , in this project ,means the local minimum cost,in my opinion, the optimal like some kind of gradient descent algorithm. In MPC part,in order to call the IPOPT 'Solve' , we need to construct the input data structure for this function.The code is showing in mpc.cpp, the length of array that contains each variable in model is determined by hyper parameter 'N',N is the length of prediction sequence, it means how many next continous steps you want to predict in time-step sequence, we talk about N later.

There are two different kinds of constraints in the model : 

1. constraints for each variable, this means the range of values for each variable.Like the 'delta' and 'a' are actuators for this project,represent the change rate of angular velocity and acceleration, 'delta' value need to set lower bound and upper bound as [-deg2rad(25), deg2rad(25)], it is said that was the simulator's steering scope, the 'a'  need to set lower bound and upper bound as [-1,1] ,1 means full accelerate,-1 means full brake.

2. constraints for model formula expression, for variable initial input value ,the lower bound and upper bound is the same,that means the initial value is fixed, no need to optimize.for others , need to set them to 0, that means the formula is an equation.In fact ,these constraints are used for 'operator' method in 'FG_eval' object.

There is some difficult to understand for FG_eval object.Follow the IPOPT and CPPAD document, I think the FG_eval is a kind of template class, is a data structure that defined to be called for IPOPT solve() method,  the operator method is main part in this class.I thought this method likes some kind of calculating map,for example , tensorflow in python. The method do not define how to value each variable, instead, it defines what the relationship between each variable.The 2 parameters of this method are CPPAD_TESTVECTOR type, one for calculating the cost and describing the relationship of variables,called 'fg' , one for transfer the initial values called 'vars'.fg[0] is used to define and sum up the cost, and the main target is minimize the cost.For the cost part ,I defined 7 cost constraints :


- cte,epsi:minimize the difference of distance and angle from the current route to reference route.
- velocity :minimize the difference of current speed to target speed, in order to keep the car in a relatively stable velocity, and prevent the car stopping when cte,epsi satisfied the minimum condition.
- delta,a :2 actuators, to keep them minimize their use:economic,necessary and smooth;
- the gap between sequential actuations: minimize the change rate of delta and a , that can make the car driving more smooth and stable.

Compare to the quiz 'mind in line'. There is a little difference is that, I set 2 weight factors to enhance the
cte and epsi cost, because the cte and epsi cost values are too small compare to the others, that means the cte and epsi optimal target would be ignored if the other cost values increase hugely ,so the weights need to balance.The main rule to set the factors value is to keep each cost constraint in a same order of magnitude.

For setting the expression of variables , I just follow the quiz 'mind in line' to build the kinematic model.One place is different is that , the simulator runs in a curve route, not a straight line , we need to define f(x) expression as 3 order polynomial and defined psi(dest) as 3 order polynomial derived function f'(x).Our eventual aim is to get the proper actuators in next time-step via MPC, so we just need to define the the delta and a at time t, not t+1.  I defined a struct type varibale to return the result from MPC Solve method, the struct type variable contains predicted trajectory and the actuators.


#### 2.Choosing N and dt for MPC model
N is the steps of predicted sequence.dt is the duration of actuators launch control instruction.That means
N*dt (let's say it as T)  represents the full predict duration in seconds.It also  can be understood as a distance that the car might be driving along in this duration.So if the T is too large , it implies we predict the car has a long way to go along with the reference route, obviously it is unrealistic in real world, too long prediction duration would not make sense for future . So firstly,we need to estimate the range of T. I understand this question by this way: in our driving experience, we decide how to control the car's motion is depending largely on the visibility,that means if we have a good visibility we can finetune the car ealier,oterwise we should slowdown the car's speed to buy time for motion ajust.I think this also works in selfdriving car, the path planning module or sensors will send back the planing route positon, and fit this positon to a polynomial,that is the visibility of selfdriving car.So we need to know a proper polynomial length could work in most cases, so that the prediction T duration can fit it with no "overfitting", and the result of the proper length divide by a proper velocity will be the T duration. Along this mentality, I need to calculate the length of polynomial for each time-step ,and observe the length in order to know their distribution in most cases. 

![alt text][image1]

To calculate the length of 3 order polynomial, I used the formula of 2 order polynomial integral derivation instead, because the coefficient of the 3rd order is very tiny ,near to 0 in this project, on the other hand the integral derivation for 3 order polynomial is very complicated.I think that can estimate the length of polynomial for most cases. I trun down the speed ,and set a group of N and dt initial value ,just make the car can drive and observe the average length,minimum length and average speed value. For most scene before the car off road, I got the average length of polynomial as 42m, and average speed about 24m/s(when I set the target speed as 70mph).The minimum length as 28m for a long time(maybe this value not the final result ,because the car usually off road at some sharp curve) , at that time,the speed as 20m/s.Consider there is a latency in sysytem, that means when I receive the send back data, the car has already driven for a latency duration. So for calculating ,I need to minus this value,the formula approximately like this :

length(average) - latency * speed(avg) = T*speed(avg) (for average case); 

length(minimum) - latency * speed(in that case) = T*speed(in that case) (for lower band case); 

Let's assume the latency as 100ms,that we can calculte the T=N*dt in a range [1.3,1.65].That is just a approximate estimate. It need to try and finetune many group of combo of N and dt values. 

After that I finetune N and dt. I think the dt could not be too small, because in 70mph situation , too small dt means the car would ajust itself's state frequently , that would make the car shake sharply in high speed situation, and that value can not be too large , that would lead "discretization error". The N value must enough big so that it can evaluate the state for whole duration with less errors.I tried the group values of N = 12,dt = 0.13,N = 10 dt = 0.16 ...N = 8 dt = 0.2 , I observe the car's performance and the print information of state, finally I choose N = 8,dt =0.2 , the car can perform a smooth and stable drving in that combo parameters.It looks perform better than working with PID controller. 

I'm not quite sure of estimating the duration T in a right way, but for the parameters :N = 8,dt =0.2 really bring a better performance.


 
#### 3.Polynomial Fitting and MPC Preprocessing
In polynomial fitting part, Firstly I transform the reference points which represented by global coordinate to car's coordinate. we knew how to transform the car's  coordinate to global's,for this project , it just a inverse process .We can deduce it from :

![alt text][image2]

so the xc,yc can be expressed as:

	xc =sin(theta)*(ym-yp)+cos(theta)*(xm-xp);
	yc =cos(theta)*(ym-yp)-sin(theta)*(xm-xp);

I define a function called "get_displaypoint" to work with it(see in main.cpp). After that, I call the function "polyfit()" to fit a 3 order polynomial , 3 order polynomial can fit the road for most cases. Working in car's coordinate is convenient for both MPC state initialize and coordinate display, and I would like to do MPC state initialize in car's coordinate.


#### 4.MPC with Latency and Preprocessing
 
I think the most tricky part in this project is here:

Since we transform global to car'coordinate, the car's position always at: x = 0,y =0 ,heading angle= 0;The latency,which means when receive sysytem state information ,the system has already run in this state for a while.So, when we consider the system current state ,we need to add the latency to sysytem, we need to calculate the system state after latency and estimate the latency state in car's coordinate,in other words,we need to calculate the difference between the state in ideal(no latency) and the state in latency by car's coordinate.So we need to do MPC preprocessing within a latency.

Here is my state variable list:

		double psi_dest // the change rate of angular velocity
		double latency // the latency of system
		double x_lt // the x state after latency
        double y_lt // the y state after latency
		double psi_lt // the heading angle after latency
		double v_lt// the velociy after latency
		double cte_lt// the cte after latency
		double epsi_lt// the epsi after latency

I defined the variable 'psi_dest',that represents 'ψdest', the change rate of angular velocity, psi_dest should be arctan(f'(xt)) ,as we know the derivative of 3 order polynomial as:

f'(xt) = (ax+bx^2+cx^3)' = a+2bx+3cx^2 

consider the origin point x = 0 in car's coordinate,just as x(t) =0,bring x = 0 into f'(xt),psi_dest = arctan(a);Following the instruction of this project, the steer angle has an opposite direction compare to the formula used in lesson.So ,I think I should set the psi_dest as -atan(coeffs[1]).

I set the latency to 100ms ,just as I assumed in "choosing N and dt" part. I also tried other values ,It seems performed not good.

The 100ms duration is very short, we can think the car drives in a stright line,that means car drives along x-axis in cars'coordinate,  So the x_lt as v*latency; y_lt as 0;

As heading angle = 0 in ideal state in car's coordinate,follow the formula:

 ψt+1 = ψt+vt/Lf *δ *dt

the psi after latency as "psi_lt". the delta is angular velocity represented by negative "steer angle" that received from sensor.  psi_lt = v/Lf* delta * latency;  

follow the formula for Cte(t+1) , as yt = 0,the Cte(t) = polyeval(coeffs, 0), the formula can express like this:

cte_lt = polyeval(coeffs, 0) + v * sin(psi_dest)*latency;

The orientation of these confused me a lot,I think maybe the opposite direction of the steer angle make things complicate.

As epsi(t) = psi_dest (psi is 0 in car's coodinate),the epsi(t+1) = epsi(t) + v/Lf* delta * dt.

I input these state value to MPC model, and get the result of actuators and trajectory from MPC solve method.Follow the tips for this project ,I set negtive value to the steer angle.


#### All of above I commented in code. Please check it for details.
  





 

