# **Path Planning Project** 




[//]: # (Image References)
[image1]: ./finite_state.png "finite_state_machine"
[image2]: ./nearcars.png "near cars"

### The project information and code running dependencies link : [project info](https://github.com/udacity/CarND-Path-Planning-Project) 

#### 1.Overview
The code frame I used was Aaron Brown and David Silver had introduced in the walk through lesson video.I think Aaron's code frame is very good.simple and easy to understand, especially the spline tool which more friendly and easy-using than 'minimize jerk 'algorithm. Although maybe 'minimize jerk' perform better
than spline,which could provide smoother curves than spline did, and ensure a minimize jerk used. But minimize jerk need to set a 's' action over position and 't' as action over time, besides this,it need to calculate the coefficients of quintic polynomial s(t)) every time.So in this project ,I mainly foucus on how to build a proper planner,in The following paper I 'll mainly talk about this issue. 

#### 2.Finite state machine design  
I think there are mainly 5 basic states in highway driving mode.
Keep Lane(KL) ,keep lane and deceleration(KLDEC) , lane chaning left(LCL)，
Lane changing right（LCR）and initial state (IS), the IS state basically is one direction state change, since the car start up to drive from stop state, and there is no stop cretia.So in the KL or KLDEC state ,there is no limit for next state choosen, it could be one of these states except initial. But for LCL or LCR state ,we don't allow they repeat their state for next time, partly because it's not safe and comfortable, further more, it could cause a max jerk error, that means it is not feasible. Based on the above state analysis, I start to design my   self drive car path planner.


 ![alt text][image1]


#### 3.Planner decision
In my opinion, the planner decision  procedure basiclly like this: the ego car will keep the KL state until it detects some obstacle objects in front of it in a relatively short distance.Then the car's planner need to make a decision : turn right ,left or just deceleration to avoid collision. So we need to know these things : how far we set a valid detecting distance;what motion state the cars near ego car, how to measure the cost of each possible state we could choosen in next time,and where is the safety boundary.I think above is a quite simple and mainly scene in highway driving, but it can not cover all the situation exactly.In my practice in most of time it can run steadily and safety.To handle these issue handily, I create a class called 'get_state()',you can check the code in 'get_state.cpp' and 'get_state.h'.the class mainly function is calculating the cost and making a decision to choose the next state. 



#### 4.Gather the cars' information from sensor fusion
To get the cars's information that we need provide to the planner, I designed a method called 'get_nearest()', this method returns at max 6 cars which around ego car(like the graph below). In each lane there are two cars, the nearset front car and the nearest behind car which distance measured by 's' component in frenet coord.So if the front car in ego lane is near ego car( that was measured by the speed difference and the safety buffer time) , it is time to change the KL state.Otherwise the car will keep the KL state and driving along the lane. 

 ![alt text][image2]


#### 5. Cost function design
For cost function design part , I mainly consider these factors ,the distance between goal lane front car and ego car; the lane changing safety gap; the speed of front cars ,and the extra turning cost.I used a sigmoid function to generating cost , the function like this: 

```
2.0/(1+std::exp(-x))-1 
```

The distance cost I build function called `dist_cost()`,
the far front car from ego car ,the smaller the cost , for more details ,you can check `get_state::dist_cost(double dist_s)` function. For the speed cost part,  I consider two factors ,one is the speed ,the other is distance,that because if front car has a long distance to ego car,even if the car's speed is slow,but the distance is large enough to do other options ,we should leave out of account for its speed .But one more thing we need to note is that, we should set a proper  lower weights to speed cost , because in same situation(same distance) we choose changing lane state need the front car in goal lane has a much faster speed.The lower weights to speed cost ,the margin of speed between two front car is larger. The extra turning cost ,since the lane changing has some  potential safety hazard , I add some punish on it,it expresses like this :

```
(logistic(1/front_dist)+logistic(1/behind_dist))*0.5;
```

For the lane changing safety gap part, I set it as a fixed number: 

MIN_SAFETY_GAP_FRONT =15 ,MIN_SAFETY_GAP_BEHIND = -15

that means lane changing must ensure the gap larger than 30 meters , otherwise , the cost would be a big number(here I set to 1000) to prevent any lane changing.I think that can be partly improved in the future, which set the gap value related to the relative speed.

#### 6.Other optimal 
There are several optimal I have made in cost function.Besides distance cost,speed cost and turning cost, I also add some other type cost to optimal the ego car's driving.


1.efficiency cost ajust, that is mainly consider in some situation,the car could be 'stucked' in edge lane(lane 0 or lane 2),in this situation,the car should change to middle lane if it meet some criterias like:go straight and turning has closely cost ,and the third lane is free.it is mainly to prevent the car was 'locked' to following the front car on the edge lane,even if the third lane was free and has a chance to turn to middle lane.you can check the code in `get_state.cpp` in line 123 to 148. The mainly thought is try to balance the cost between the heavy traffic lane and the free traffic lane.

2.middle lane prior. At the same situation(means both middle lane and the ego car driving edge lane are free ), I prefer to choose the middle lane,because in middle lane we have more options, it is more efficient driving in middle lane in same circumstance.

3.successive change lane avoiding. I defined a global variable called `cl_extra_cost`,mainly because two successive change lane operations in short period would cause max jerk warning.
the jerk warning means this kind of operation maybe not feasible or safety and comfortable. 
So I need add a extra punish on that to avoid successive change lane operations,`cl_extra_cost` is a extra cost of lane changing action.

-----------------------

### The following content is a reflection on how to generate paths.



[//]: # (Image References)

[image10]: ./spline.png "transform"
[image20]: ./insertpoints.png "insertpoints"
[image30]: ./jerk_point.png "jerk_point"


#### 1.Model selection
There are several generating path method that we could use in this project:
spline,polynomial , minimize jerk algorithm and so on.The main issue path generating discussed is how to find a smooth curve fitting points in path and make the jerk in a acceptable scope. Here I choose spline as generating path method.Because the spline method simple and easy to understand and use, although maybe 'minimize jerk' perform better than spline,but it need to calculate the coefficients of quintic polynomial s(t)) which I have to code a solver to do it.


#### 2.Spline features and generating path in this project
Spline is a piecewise function, that can ensure each points in the path can be fit in it, it was continuously differentiable, so the function trace was smooth.In this project, we use 5 points to generate the spline .the first 2 points gain from the last 2 points which not been excuted by simulator in previous cylce. By the first 2 points we can get the heading information of the ego car.After that ,we gain the other 3 points which in ego car's current lane with some distance interval expressed by frenet coord, then we transform the 3 points to global coord.I think  the longer the curve spline function generated and proper sparce points we picked can make the curve more smooth and low curvature, which is very important for a car driving in a feasible path. After the 5 points was expressed by global coord , we transform the 5 points to a car's sight coord refer to the first of the 5 points, what we have done in lesson 'Particle Filter' Term 2 ，then we fit these 5 points to spline, the example spline like below.

 ![alt text][image10]

#### 3.Insert the points to the spline curve
Once we have generated the spline curve,we need to insert points to the curve ,these new points will attach to the points which not excuted in previous cycle.In fact , we only need to add several new points in each planner cycle, most of the points in plan path list were reused from previous cycle.By this method, we can get a more smooth and continuous path. In insertion points phase, we calculate the ego car's driving distance in 0.02 s(the simulator excute cycle was 20ms), and we set a target point, the length of target distance at least need to meet this cretia:

linear distance > count of news points * driving distance in 0.02 s;

the target point linear distance to the car divided into N parts, and this mapped to x-axis caused a series x values, by these x values, we can solve these y values by spline function, thus we get the coord of new points.But remember, the (x,y) of new points we 've got was in a car's sight coord. we need to transform them to global coord later.  

the example graph see as below,  the target point would not too far from the reference point, because here we used a linear distance to approximately equal to the real length of this curve, if the target point too far from the reference point would cause larger errors, that may cause the car's driving trajectory not smooth as assumed. When we finished insertion points , we transform these points
In a sense, the mechanism of generating path by spline function is similar to MPC controller, each time we generate a curve, and only pick the first several points, and iterate the procedure.

 

 ![alt text][image20]

####4. Max jerk error in some situation and solution

Although the spline method is awesome, in some situation it would still cause max jerk errors. I found it would cause max jerk error in successive lane changing scene. I think mainly reason is when the ego car received a second lane changing instructions , the ego car has still in the previous path and the first lane changing action has not completed yet.like the gragh below , I think these two points may cause jerk error.To avoid this kind of errors, we could set a punish cost to successive lane changing scene.In practice ,I found there are at least 2 'KL' or'KLDEC' state intervals between 2 successive lane changing action can erase the jerk error.




 ![alt text][image30]
 
		


#### I commented the codes in every .cpp file,please check it.


---------
**email** :*zooboy@vip.sina.com*

 

