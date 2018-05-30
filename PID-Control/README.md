#**PID-Control** 




[//]: # (Image References)

[image1]: /PID.png "PID"

### The project information and code running dependencies link : [project info](https://github.com/udacity/CarND-Controls-PID) 

####1.Understand the PID-controller and How each of the PID components effect 
 PID stands for Proportional,Integral and Derivative, 3 separate parts joined together, to ajust the parameters (like steer, speed,temperature and so on.)   of process control converge on the target value effectively.The PID controller was widely used in industry field.

In my opinion, the proportional part used a coefficient (kp) directly calculated with the errors that the system sensor detected.It has a decisive and intuitional  effect on decrease the errors,but it also has a negative effect on increase overshoot, it would perform like a wave along the target value if we only use P controller.

In order to decrease the overshoot and make the wave amplitude more convergence,scientists introduced the derivative part.The derivative part is a differential of error. In practice , it can be expressed as a difference value between two time-step.The main reason that 'D' part can decrease overshoot is that:the differential part represents a trend or predict of the state,that means if the differential of error is positive, it indicates the error was increased,so in next time-step,it should make a additional reduction to compensate the increasing error,and the additional reduction part can be expressed as coefficient (kd) * differential.So the D part can decrease the overshoot by affording a additional compensation,obviously it can decrease the time to meet equilibrium state.


Assume the system has a bias that make the error grow up more and more in each time-step,how to eliminate it ? For solving this problem, the I part has been introduced to the controller.The I part is a intergal of the errors expressed as coefficient (ki) * intergal(the sum of errors of history at particular time-step).This part is imported for compensation the errors that caused by solid bias of system.And in practice, almost all control systems alway have a bias even the bias is very tiny. That means in an ideal no bias environment, the PD controller is good enough to get the convergence target, but in practice, the bias as a noise should be against by measurment the total errors that what the I part did.

 ![alt text][image1]

As the graph shown above. the I part can also increase the overshoot,and consider the I part is a intergal result, the coefficient ki must small enough.

In this project, increase kp can make the car converge to the target position quickly,but it also increase the overshoot hugely, and off road finally.Increase kd means add a bigger weight to D part , it can reduce overshoot dramatically and make the run more stable,but if kd value too higher, it could make the car steer too sensitive to keep driving along with a smooth trace.I think the main reason is too high weight with the D part would cause the car repeated adjustment its angle when a slightly overshoot happens,because the D part perform an oppsite direction to the overshoot,that makes the car oscillate along the reference line.
The I part need to be small enough,because the intergal would be huge, and the product of ki and intergal must keep in a proper range, otherwise it would wind up and cause seriously overshoot.But it indeed important, because the car driving in a variable accelerated motion state and the reference line not a straight line too, so I think these may cause a lot of bias , we must reduce the errors caused by bias.
 

####2.How I choose the coefficient
I choosed the coefficient manually, I have noticed some students used twiddle to find out better coefficients,and they post their solutions on the forums.The twiddle is good,that is no doubt.But even in twiddle mode , it also need a lot of time to iterate each coefficient. I have also noticed a paper about PID controller  written by George Gillard, in this paper he recommended a fine tune method called ' Ziegler-Nichols method',but I think how to confirm a proper 'ku' and 'pu' is a relative subjective judgment, it not easy for me to do that.

For manually choosing these coefficients, first thing we should consider is that what the proper range is for each coefficient . the P part is a proportion of error, obviously the kp should in range (0,1), I set kp initial value as 0.3. The D part can restrain overshoot, I set it to 5 as initial value. And for the I part , the ki need to small enough because the whole part is a product of  intergal and ki.I set ki to 0.005 as initial . Then I set the throttle value as 0.5, the throttle value in my thought is car's power output ratio. the higher throttle value the higher the car's speed. I print the debug information for each 100 time step to observe the mean absolute cte value performance. For the initial value , the car can drive over full lap, I got mean cte score around 0.40,it has a little shake even drive along a straight line. I turn down the throttle to see what if the car in a low speed mode, it perform very good, the car drived more stable and more safety,and the mean cte score lower too, nearly 0.25.It shows lower speed has more stability than high speed.But I decided to use throttle value as 0.5 to fine tune my controller, because  the car driving speed around 50kmh is more close to the real world. Based on the initial value, I first fine tune kp coefficient by a step as 0.1 , I tried 0.2,0.4,0.5,0.6. The result shows   the mean cte is better than other value when kp = 0.5.Then I finetune kd with a step as 5.Theoretically , the higher the kp is ,the higher kd value the system need.Because they have a compensation relationship in overshoot.I tried 10,15,20. It shows the mean cte is better than other value when kd = 10,then I lower the ajust step to 2,I tried kd = 8,10,12,14, finally the kd set to 10.Then I ajust ki by step 0.005,I tried 0.01,0.015,0.It shows ki higher than 0.005 whould lead a worse result. then I lower the step to 0.001, I tried 0.001,0.002,0.003,0.004,it seems ki = 0.001 has a better result.

Generally, I finetune these coefficients use the method like this: first choose a group of initial value, then finetune each one(the order is kp,kd and ki) by a bigger step, once I got a better result, smaller the step to search around the roughing value to get a accurate  value.   

So far, I determine kp,ki,kd as 0.5,0.001,10. The performance is not satisfactory as I imagined.It has obvious shakes when the car cross curve.However the car can run over laps with no off road and has a mean cte score around 0.28(statistics over 6000 time steps ),I think that may be the best result in this situation (throttle fix to 0.5,no other finetune with speed )


####2.Some points could improve in future 
In Sebastian's lesson, I think there are 3 points that different from the project. 

1. the model has a constant speed but in this project the car's speed is a variable.

2. the reference line in lesson model is a straight line but in this project the reference line not alway a straight line.

3. it would not off road in lesson model,but in practice it would.

Based on these 3 different points.I think there are several places could improve:

1. add a new PID controller for throttle to make the speed stable on a target.
2. consider degrade the speed when car drive on curve road.(the lower the speed is the lower curve negative effect.) 
  





 

