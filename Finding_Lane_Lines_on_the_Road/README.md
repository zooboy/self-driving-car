# **Finding Lane Lines on the Road** 






**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report


[//]: # (Image References)

[image1]: https://github.com/zooboy//test_images_output/solidWhiteCurve_piped.jpg "solidWhiteCurve"
[image2]: /test_images_output/solidWhiteRight_piped.jpg "solidWhiteRight"
[image3]: /test_images_output/solidYellowCurve_piped.jpg "solidYellowCurve"
[image4]: /test_images_output/solidYellowCurve2_piped.jpg "solidYellowCurve2"
[image5]: /test_images_output/solidYellowLeft_piped.jpg "solidYellowLeft"
[image6]: /test_images_output/whiteCarLaneSwitch_piped.jpg "whiteCarLaneSwitch"
---

### 1. build the pipeline


The pipeline consisted of 6 steps：

1. converted the images to grayscale;
2. used the function gaussian_blur()(kernel_size=5) to smooth the pictures;
3. used the function canny(blur_gray,50,150) to find the edges of pictures;
4. I choose the mask region as ([430,330],[150,540],[900,540],[540,330]),It's a quadrangle;
5. In Hough transform section,I pick the parameter as: rho = 2 ;theta = np.pi/180; threshold = 15;     min_line_length = 70; max_line_gap = 40;
6. uesd weighted_img() to draw lines on pictures

I tried many combos of parameters, and finally I chosen these like I said above.

In order to draw a single line on the left and right lanes, I modified the function hough_lines(),add a  return value of lines.I defined the function  lines_sort() to detect the right lane and the left lane by their line slope value.Then,I defined the function getfulllines() to compute the average slope and 
intercept, then according to the linear equation return the lines extrapolated top and bottom of mask-region.After that, I picked these lines that getfulllines() returned to the function draw_lines().in order to make the lines semi-transparent, I mixed the drawed one to the original one, by weighted_img().

I have tried the optional challenge,but failed to pass it.I thought the quadrangle  mask-region is not fit for it.and the curve lane need some other method to detect and draw lines.


![alt text][image1]
![alt text][image2]
![alt text][image3]
![alt text][image4]
![alt text][image5]
![alt text][image6]

in ```test_videos_output``` folder you can find the test video files  

### 2. Identify potential shortcomings 


Firstly I used the arithmetic-mean of the slope and intercept as their average value,but I found it was not make sense just because some tiny lines as noise were not in the right direction, and their slopes were involved equally to the average value.So, I modified the getfulllines() function,computed the slope and intercept average value by weighted-mean based on the lenth of lines.

Compared to the old version getfulllines(),the new one draw-lines performed less tremble in the video.But,it still had a little trembled when the car tended to make a turn.





### 3. Suggest possible improvements for the future

1. some nor-linear condition,like curve, not suited for the Hough Transform line detect mode, need add some polyfit  method to draw lines.
2. the quadrangle mask-region may not suited for all sense,consider some self-adaptive method to detect the mask-region.


---------------------
**email**:***zooboy@vip.sina.com***
