## Advanced Lane Finding



**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./writeup_paper_image/undistorted.png "Undistorted"
[image2]: ./writeup_paper_image/test2_und.jpg "Road Transformed"
[image3]: ./writeup_paper_image/test3_thr.jpg "Binary Example"
[image4]: ./writeup_paper_image/pertrans.png "Warp Example"
[image5]: ./writeup_paper_image/test7.jpg "Fit Visual"
[image6]: ./writeup_paper_image/final5.jpg "Output"
[image7]: ./writeup_paper_image/ajust7.jpg "Finetune Output"
[image8]: ./writeup_paper_image/formula.png "formula"
[video1]: ./writeup_paper_image/project_video.mp4 "Video"

 

---

### README


### Camera Calibration


I used the chessbord pictures that provided in this project in folder ./camera_cal, Here I am assuming the chessboard is fixed on the (x, y) plane at z=0, such that the object points are the same for each calibration image.  Thus, `objp` is just a replicated array of coordinates, and `objpoints` will be appended with a copy of it every time I successfully detect all chessboard corners in a test image.  `imgpoints` will be appended with the (x, y) pixel position of each of the corners in the image plane with each successful chessboard detection.  

I then used the output `objpoints` and `imgpoints` to compute the camera calibration and distortion coefficients using the `cv2.calibrateCamera()` function.  I applied this distortion correction to the test image using the `cv2.undistort()` function and obtained this result: 

![alt text][image1]

### Pipeline (single images)

#### 1. Distortion-corrected image.

To demonstrate this step, I will describe how I apply the distortion correction to one of the test images like this one:
![alt text][image2]

#### 2. Binary image

I used a combination of color and gradient thresholds to generate a binary image  Here's an example of my output for this step.  (note: this is not actually from one of the test images),besides the gradient direction\magnitude\ absolute x or y axis gradient\  HLS color space
threshold,I add a RGB color space threshold in order to enhance the white and yellow color recognition. 

![alt text][image3]

#### 3. Perspective transform

The code for my perspective transform , which appears in lines 329-340 in  `Advanced_lane_line_finding.py`.
Consider one image maybe have errors,I choose two images of straight lanes and make a average of the points 
the source code like below: in this code cell I get the transform and reverse transform coefficient.



offset =300

src1 =np.float32([[579,460],[703,460],[1115,717],[194,717]])

src2 = np.float32([[585,460],[703,460],[1090,718],[234,718]])

src = (src1+src2)/2

dst =np.float32( [[offset,0],[1280-offset,0],[1280-offset,720],[offset,720]])

M = cv2.getPerspectiveTransform(src, dst)

Minv = cv2.getPerspectiveTransform(dst, src)

I measured and checked the transform points in the 'Windows10' Painting accessories software,this software can
draw straightlines and show the x,y coordinate in pixels,thus I measured these two images,then tried several times,finally got these satisfied bird-eye perspective pictures. 


![alt text][image4]

#### 4. Fitting with a polynomial

I choosed the window sliding search method to fit these valid pixels to fit a 2nd order polynomial,the code based course video and I finetune some parameters.you can see the code in lines 372-440 in `Advanced_lane_line_finding.py`,the method called`windowseach()` and the method will return values like :

out_img:the image of lane detected and marked by two different color,and if the `LinesCombined==True`,it will add polynomial curve on it 

left_fitx:the left lane's polynomial x axis points

right_fitx: the right lane's polynomial x axis points

right_fit: the right lane's polynomial coefficient list

left_fit:the left lane's polynomial coefficient list

ploty:y axis points

leftdots: the amount of left lane's valid coordinate points

rightdots:the amount of right lane's valid coordinate points

I choose the ' min pixel threshold ' method to apply the sliding window search.
And in contiuous frame , the search will based on the reslut that sliding window search provided(see in code lines : 754-784,`autosearch()`method),in  contiuous frame search mode,the line searching will based on the lines' position in last frame image.

But the result looks not very satisfied.the two lines are apparent not parallel in some images,so I thought it need some finetunes.

![alt text][image5]

the finetune method called `ajustlines()`,consider the finetune would be used in continuous frame search mode,I set the `isWindowMode` parameter as a switch variable(there is some difference between sliding window search mode and continuous frame search mode,you can see the notes in `Advanced_lane_line_finding.py` or `Advanced_lane_line_finding.ipynb`,I prefer the .ipynb file ,because there are also markdown comments in it )

the the principal is : 

* pick the line which has more lane pixels points as the reference line(more points mean more accuracy in polynomial fit) .I think it is very important to judge the line whether straight or not,because the straight line has a very large scope in radius,theoretically it could be infinity and that means can not put the radius change rate to judge whether two lines are similar parallel when they are a pair of straight lines.So it need to distinguish.
* 
if the reference line close to a straight line,ajust the other one to straight lines with same coefficient in window sliding search mode,and nothing to do in continuous frame mode  (consider the car pass large scale corner the camara would have some time delay to capture two lines strict straight pictures,that means it is reasonable one line straight and the other one not in continuous frame mode  ).
* 
if the lines not straight, and the lines's radius difference rate in no-finetune threshold scope, no need to ajust.
* if the lines not straight, and the lines's radius difference rate out of no-finetune threshold scope, ajust non-reference-line based the two lines' mean polynomial coefficient.
* 
in window sliding search mode, if the ajusted-lines's radius difference rate still out of max-threshold scope(it would imply some lines not correct identified in search step or not extract enough valid information in thresholded binary step),use the reference line polynomial coefficient value(except constant value) to instead of the other line's.In continuous frame mode,it maybe imply this frame is a bad frame,it would be dropped,so no need to ajust.
![alt text][image7]
 
#### 5. Calculated the radius of curvature of the lane and the position of the vehicle with respect to center

I did this in lines 675-698 in my code in `Advanced_lane_line_finding.py`,the method `car_position` caculate the car's position to center, and `print_radius` print the radius information. these two method will be called in method `drawlines()` code in lines 716-744，I calculate the radius of curvature based on this formula :

![alt text][image8]

use the method `curverad`code in lines 467-482 , it will return the result refer to meter if the siwth variable `refmeter` is True,to accurate calculate the  radius of curvature in meter ,I need measure the width of lanes and length of dash in pixels as the meter-pixel transform reference.

I calculate the car's positon by assuming the camara in the middle of the car,thus the image center is also the car's,so if I get the margin of center of the car and the lanes then change them in meter, I can know the position information.


​​
​​

#### 6. Example image of result 

I implemented this step in lines  716-744 in my code in`drawlines()` in `Advanced_lane_line_finding.py`.  Here is an example of my result on a test image:

![alt text][image6]

The `drawlines()` method called `cv2.pullText` method to print the information on the image.

---
##### More information you can see in the `Advanced_lane_line_finding.py` file,I added comments in every code cell.

To easy the work of pipeline's intergrate and make the program easy to debug  and variable easy to manage. I defined 2 class-type variables, class `line()` and class `imgprocess()` ,class `line()` record the line's information(see in code lines 504-566) ,class `imgprocess()` record the current image's information include each step's result and the lines in the image,it intergrates each process step method,besides this it has a private method `__comparelines()` to compare the current lines that found in current image  to the lines in last frame image,the decide whether to record it or drop it. 

### Pipeline (video)

#### 1. Final video output. 

the video link here：  [[link to my video result]](./writeup_paper_image/project_video.mp4 "project video")

if the link not work ,please check it in `/writeup_paper_image/` folder,
and the example output images also in this folder.

---

### Discussion

#### 1.  Problems / issues you faced in implementation of this project


1.I think one of the difficult is to find a combo of threshold method and values in binary image lane information extract step,it is not complicate in thoery,but it need lot of practice to find better one. the principal I think is to extract necessary information in each threshold scope and reduce the noise as much as possible, because their relationship is 'OR' that means any noise would be added in the final result.in order to enhance the white and yellow lines performance,I also add a RGB color space threshold.

2.The other problem is how to catch and finetune the lines polynomial equation precisely, in this project I used ' min pixel threshold ' type  window sliding search method,and I think it is not very accurate, and the course also recommended a convolution method to catch the max points position,I think maybe the convolution method better than ' min pixel threshold ' method,but it still has defect, imagine if the frame image has a dash lines in it,so we can not get more information on the lanes (so little points information ),and if there are some noise in it,it would be indenty wrongly.I think there is no perfect method, we need a method to check and finetune the polynomial results,I think this is really difficult point,the difficult is that ,need to consider many different situation,like how to check the two lines parallel,and if they are not parallel,how to ajust them,how to examine the ajust is reasonable,neither underfit nor overfit,how to ajust them when the road is not flat ,and so on.
Besides this, I think there is a little different in finetune between two search mode(windows slide search and continuous frame search),because the window slide need more accurate and it will become the reference for the future frames,so it need more strict ajust.How to different the two scene is one of the problems. 

3.To enhance the robust of the program,I think there are some points can try:

* Enhance the perspective transform image's clarity,some images trend to blur when they are transformed to bird-eye perspective, and this blur tend to much noise in binary image.
* Consider how to enhance the image threshold method,to filter more noise like tire tracks.
* Define more criteria to evaluate the lines whether good extracted or not.
* If the lines not parallel and similar in real world,consider how to confrim them.
*   Consider some other equations not only 2nd polynomial to fit the lane when the lane points seems not standard in some special condition.