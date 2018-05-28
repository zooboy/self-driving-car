#**Traffic Sign Recognition** 



---

**Build a Traffic Sign Recognition Classifier**

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images



[//]: # (Image References)

[image1]: ./examples/visualization.png "Visualization"
[image2]: ./examples/grayscale.jpg"Grayscaling"
[image3]: ./examples/p1.jpg "traffic-signs"
[image9]: ./examples/v2.png "Visualization-2"
[image10]: ./examples/showimage500.jpg "Show-image-1"
[image11]: ./examples/showimage2.png "Show-image-2"
[image12]: ./examples/modelconfig.jpg "Model-config"
[image13]: ./examples/error.jpg "error-rate"
[image14]: ./examples/errorchart.png "error-hist"
[image15]: ./examples/prob.jpg "top-5 prob"
[image16]: ./examples/filter.jpg "conv1-filter"
[image17]: ./examples/m.jpg "origin"
[image18]: ./examples/webpred.jpg "web-pred"


---
### README



The project code  see in  .ipynb file.

###Data Set Summary & Exploration

####1. Basic summary of the data set.

#####I used the numpy library,basically used the 'shape' attribute to calculate summary statistics of the trafficsigns data set

The training dataset comes from [German Traffic Sign Dataset](http://benchmark.ini.rub.de/?section=gtsrb&subsection=dataset).

* The size of training set is 34799
* The size of the validation set is 4410
* The size of test set is 12630
* The shape of a traffic sign image is (32,32,3)
* The number of unique classes/labels in the data set is 43

####2. visualization of the dataset.

Here is an exploratory visualization of the data set. It is a bar chart showing how the data  ,I designed a function to show the data ,the function called 'static_show',It had 3 parameters:the data,the label, and a bool variable callednorm (whether to show in normalization).In the normed mode,we can compare the every class ratio of trainset,validset and testset. Besides,I made a mapping between the class id to the description,and put it as the x-axis in the plot.

![alt text][image1]
![alt text][image9]
###Design and Test a Model Architecture


At first step,I need to design a function to show the images,and not only show the image itself,but also can output the info of these images.the function like this:'show_image(index = None,X =X_train,y=y_train,showgray =False)',the 'index' means the images's index, default is 'None',then we would pick it randomly.the X is the  data and the y is the label part.the label to use show the detail info of signs.like this:

![alt text][image10]

The bool parameter 'showgray' is used to show grayscale when the image is  processed to grayscale.Secondly,grayscale and normalize the images,I defined a function to do it.The grayscale can reduce the layers of image, and enhance the computing ability.And in most country, the traffic sign can be distinguished by the graph  the sign drawn and the contrast of differnt color rather than by the color alone (consider someone who colorblind).So, I thought that's fine to grayscale the images.In the function 'gray_norm()',I used the cv2.cvtColor to transform the image to grayscale firsly(I also tried to mean the every layer's value to get a grayscale one,but the result was not good,it looked wired and not clear enough.)After grayscaled.I found the value of image matrix was not between 0 to 255, it was mostly in 20 to 30,so I can't use (pixel- 128)/128 as the normed formula.Instead, I used the (pixel-min)/(max-min) to normalize it.

Finally, reshape the nomalized image with numpy to fit the train model.(after the grayscale process, the shape(32,32,3) transform to (32,32),the last dimension was dropped.and the train model need 3 dimensions )

Here is an example of after grayscaling and normalization.

![alt text][image11]


####2. Describe final model architecture
The final model refer to leNet,same as 5 layers architecture,2 layers for convolution and max pooling.1 layer for flatten, and then ,2 layers for fully connection,at last, a softmax function for classfication.The different from LeNet is that:


- much deeper of the filter depth. In conv1 layer, the depth to 20, In conv2 layer to 60.
- Add more nodes in the output of fully-connect layer: fc1 -1024, fc2-512
- In every fully-conected layers (include flatten layer) add a dropout.



![alt text][image12]



####3. Describe of model training. 

To train the model, I used the hyper-parameters as follow:
 

- lr = 0.001 
- batch size =128 
- optmizer :AdamOptimizer
- epochs = 50 

I tried many combos of these hyper-parameters,like to reduce the lr to 0.0005 ,but the performance not good.And In most condition the trainning result would stable in 40 epoches.Especially some good result(higher accuracy,lower loss ) seemed appear before 30 epoches,since then,the performance seemed worsen,so I set the epoches to 50.
The batch size I tried 400,but the valid accuray was not good,I thought the batch size to big would effect the gradient update.
In order to get better result for model save.in the trainning section, I defined two value for threshold,one record the valid acc, the other record the valid loss,and gave them initial value with: initial acc = 0.973,initial loss = 0.13;only met the two value condition (valid acc >= initial acc,valid loss<= initial loss ) the model should be saved,and then update the threshold,by this way,I got relatively better result for every time of trainning.


The final model results were:
- training set accuracy of 0.993
- validation set accuracy of 0.984 
- test set accuracy of 0.967 


 

###Test a Model on New Images


#####Here are five German traffic signs that I found on the web:


![alt text][image3]

In order to compare the prediction and the real label,I made some data analysis ,I designed a function 'get_comparedf' added the info of label ,prediction,describe and the result(right to 1,wrong to o ) to a pandas dataframe. from the result I can screen out the wrong result and caculate the error rate of every class.(detail in the .html file),like this: 

![alt text][image13]

##### Based on the data ,I drawn a hist chart:

![alt text][image14]

##### Obviously, the 'Pedestrians' sign got the highest recognizing error rate.

Here are the results of the prediction:

![alt text][image18]


The model was able to correctly guess 5 of the 6 traffic signs, which gives an accuracy of 83.3%,basically speaking, the dataset is only 6 images , in my imagination ,I thought they should be recongnized easily,the miss judged image was 'Yield',in the test set, the error-rate of 'Yield' no more than 1%.I thought maybe the manual cut make the image out of shape,and the ratio of sign in the total picture was smaller than others. 

I defined a function to show  the probabilities for each prediction,called 'get_topk_prob',it used a dataframe to install the top 5 probabilities(show in percentage) and the top 5 labels of each image.

For the first image, the model is sure that this is a Speed limit (20km/h) sign (probability of 30%), and the image does contain a Speed limit (20km/h),but the fifth top probability is a negative value,it's very odd,I need to look up some materials. The wrong recongnize one 'Yield', the first top probability is 10.05%,represented 'Priority road', the second top probability was 9.59% represented 'Yield',the two probabilities are very close.It appears the model is not quite sure which one is right. 

##### The top five soft max probabilities were:

![alt text][image15]


### Visualizing the Neural Network (See Step 4 of the Ipython notebook for more details)

I generated a series of images which in the first convolution layer after the convolutional operation.
##### The original image:
![alt text][image17]

##### Extract in convolution layer 1:
![alt text][image16]

----------------------
**email**:*zooboy@vip.sina.com*


