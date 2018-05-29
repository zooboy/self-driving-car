## Vehicle Detection Project




The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

[//]: # (Image References)

[image1]: ./examples/car_not_car1.png
[image2]: ./examples/HOG_example.jpg
[image3]: ./examples/test3.png
[image4]: ./examples/test1.png
[image5]: ./examples/bboxes_and_heat.png
[image6]: ./examples/labels_map.png
[image7]: ./examples/output_bboxes.png


### Vehicle Detection summary

There are several methods to implement vehicle detection. For deep learning NN, you can choose `YOLO` or `SSD` which based on CNN architecture to deal with bounding box target detection, even you can use a semantic segmentation NN model to detect the vehicles and outline their shape.For this project I used a conventional machine learning method:support vector machine which trained by extracted hog features to implement the project. the hog features is also a conventional computer vision method to characterize the objects' shape. The purpose of this project is to learn and understand the basic and traditional technologies of vehicle detection, and these are footstones for advanced image recognition technologies.


### Histogram of Oriented Gradients (HOG)

#### 1. Extracted HOG features from the training images.

The image train set is from KITTI, The code for this step is contained in function `get_hog_features` for extracted hog features, this function is the core function for extract hog feature. and in function `extract_features_batch` for train dataset extracted,this function apply hog,color hist and spatial size features combined.  (see in `CarND-Vehicle-Detection-master_hog.ipynb`).  

I started by reading in all the `vehicle` and `non-vehicle` images.  Here is an example of one of each of the `vehicle` and `non-vehicle` classes:

![alt text][image1]

I then explored different color spaces and different `skimage.hog()` parameters (`orientations`, `pixels_per_cell`, and `cells_per_block`). I defined the function `extract_features_batch`, this function is for training model with batch images.I make some modify from the class video's example code.I add the hog_color parameter to test and finetune the train model with different hog and color hist method parameter.To see the result,I defined a function called `single_img_features` to extract features for those to be predicted images.I observe the result by the test accuracy and the function `pipeline_test` will show the intermediate results of each step. 


#### 2. Settled HOG parameters

I tried various combinations of parameters and the hog color space ,finally I used  

The Hog parameters as follow: pix_per_cell = 16,cell_per_block as 2,orient bins as 11,color space as 'YUV',and channel as 'ALL', compare with other color space ,the 'YUV' perform better.But I found there are still some missed detection in some situation, like white cars,or a part of whole car.  

#### 3. Trained a classifier

The classifier I finally use to generate the video is trained by hog features only. The color bins features and pixel features seemed not help to improve the performance of detection.I used a linear SVM classifier to do the classify job.It performed not bad ,I mean simply observe results from test accuracy which arround 99.99%. ,I extract these features and then normalised them by `StandardScaler().fit()`and `transform()` sklearn function. 

### Sliding Window Search

#### 1. Sliding window search

The sliding window search see in function `find_cars_mod` , and a multi-scale search see in function `drawcars` ,, the multi-scale list I choose is from 1.2 to 2.

![alt text][image3]

------


![alt text][image4]


### Video Implementation


please check the video in folder `project_videos_output`



---


### Problems encountered 

1.The box position looks not stable,it shakes in a large scope,the box seems tramble seriously,even lost target when the car shakes(I think this reason it the heatmap in consecutive frames combined has no intersection, if several frame are bad when the car shaking,it could cause the pipeline failed ),I think this could get a bad frame when the car shakes,and then the classifier can not identify the target precisely,besides this ,when the two car are near close or overlap, the classifer identify them as one car.I think it should to improve in tracking field,maybe some filter can solve it ,like Kalman Filter.


2.There are still some false postives in the video sences,especially in opposite direction cars,I think it need to identify the road edge and road barrier section to distinguish the car which dirved in opposite direction.



------
**email** :*zooboy@vip.sina.com*


