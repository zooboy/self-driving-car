## Behavioral Cloning 

------------------


### Behavioral Cloning Project

The goals / steps of this project are the following:

* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./images/p1_2.png "Model Visualization"
[image2]: ./images/s60clear.png "center_left_right"
[image3]: ./images/s60resized.png "resized"
[image4]: ./images/s60flip.png "flipped"
[image5]: ./images/lossvs.png "loss-chart"


The project includes the following files:

* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network 
* README.md summarizing the results
* video.mp4 record the car driving in autonomous mode

Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing(I ran it in jupyter notebook).

[the simlator download](https://d17h27t6h515a5.cloudfront.net/topher/2017/February/58ae4419_windows-sim/windows-sim.zip)

```
%run drive.py model.h5
```
The model.py file contains the code for dataprocessing, training , saving the convolution neural network and executing the simulator. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. Description of model architecture 
The model consists of a convolution neural network (you can see this in model.py  code in line 216-234) 
The CNN model I used is based the Nvidia architecture,and I made several modified on it.

The model contains 5 convolution layers and 3 fully-connect layers.

The model includes RELU layers to introduce nonlinearity .
The data is normalized in the model using a Keras lambda layer,besides this , I add a cropping layer to cut the image landscape part to make the model focus on the lane,and reduce the training time.(see in architecture graph)

#### 2. Attempts to reduce overfitting in the model

The model contains 3 dropout layers in order to reduce overfitting (model.py lines 226-230). And I set the dropout factor to 0.4.

The model was trained and validated on different data sets to ensure that the model was not overfitting (code line 119). The model was tested by running it through the simulator and ensuring that the vehicle could stay on the track.

#### 3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually,
besided this,I used 'ModelCheckpoint' to monitor the val loss and catch the lowest loss result.I set the train epoches to 7, In my experience, over that epochs the model appeared a bit overfitting.

#### 4. Appropriate training data

Training data was chosen to keep the vehicle driving on the road.I choosed the clock-wise and counter clock-wise lap of track 1 to capture the driving behavior. I used a combination of center lane driving, view from the left and right sides of the road,and I flipped them to augment the dataset.I cropped the landscape part in the image to make the model focus on the road(see in the code line 218). In order to cut down the trainning time,I resize these images from 160x320 to 80x160 according to Paul Heraty's advices which listed in his 'Behavioral Cloning Cheatsheet' in Slack forum .

For details about how I created the training data, see the next section. 


#### 5. Solution Design Approach

The overall strategy for deriving a model architecture was :

(1). comparing these models' val loss in same dataset before overfitting.
(2). observing the models' performance in simulator.

My first step was to use a convolution neural network model similar to the project 'Traffic-Sign-Classifier' , In that project I used the LeNet model as the baseline. For comparation, I used the sample data Udacity supplied in this project as training and validation dataset.

In order to gauge how well the model was working, I split the sample data into a training and validation set,set the validate ratio as 0.2. I found that my first model had a low mean squared error on the training set but a high mean squared error on the validation set. This implied that the model was overfitting. After that,I tried the AlexNet model,the mse result looked lower on both training and validation set than lenet before overfitting.But in simulator ,the car can not pass the first curve lane.Then I tried the Nvidia model.The model looked a little wired, no pooling layer in it. But it performed good,lower loss in both training and validation set than other model I tried,and passed the first curve smoothly,stopped at the bridge railing finally. 


The final step was to run the simulator to see how well the car was driving around track one. There were a few spots where the vehicle fell off the track,to improve the driving behavior in these cases, I captured more curve and recovery scenes.
At the end of the process, the vehicle is able to drive autonomously around the track without leaving the road.

#### 6. Final Model Architecture

The final model architecture (model.py lines 216-232) consisted of 5 convolution layers and 3 fully-connect layers.

Here is a visualization of the architecture (note: visualizing the architecture is optional according to the project rubric)

I found this model had a bit overfitting when training epoches over 3 times.So,I add dropout layers to resist it and get a lower val-loss result.Besides this, consider the image was resized as a quarter of the original,I reduce the Nvidia model convolution kernel size from the 2nd. convolution layer. The kernel size was cut down to 3x3 from 5x5(see in the architecture graph )

![alt text][image1]

#### 7. Creation of the Training Set & Training Process

To capture good driving behavior, I captured the center\left\right image,and used them as trainset and validation set.But the picture size is large,and the training speed was too slow, so I resized them.
the center\left\right view of image like this(size:160x320):
![alt text][image2]

I resized these images(size:80x160) ,It has lower resolution and looks a little blurred than the original one,but clear to see the lines,edges and the textrue of the lane,resizing of the images really enhanced the training speed observably,then I filpped them to augment the data point,it also helpful for model to learn the right and left sides both balanced

![alt text][image3]
![alt text][image4]

Then I repeated this process on counter clock-wise on track one in order to get more data points.

I trained the model,observed the simulator performance,and foucused on some  scenes like driving off lane,touching the lines or edges ,in order to capture more data points,and repeated this process until the car can drive stable and look safe in the lane.

After the collection process, I had 25464 number of data points.

In data process part,firstly, I  need to read these image and set the steer value of them(see in code lines :94-107).I do this process by:

(1).read the csv flie, and split the lines as two parts: part 1: the 'src.' of image(3 sorts: the center,the left, the right),part 2 ,the steer value. I set the steer factor as 0.2. 

left image steer value = center steer value+ 0.2,

right image steer value =center steer value- 0.2,

(2).output  a 2-dimensions Python List.the first dimension store the image file address, the second one store the steer value according the image.After that,I shuffle it.


Considering the image data points  huge RAM occupied could exhust computer memory.I used a generator to generate the data batch by batch.(see code lines:126-153 ),image flip and resize also processed in this generator section. 

I used pyplot.imread method,to get a 'RGB' format image array.Because I found the pyplot.imread has a different color format with cv2.imread(),the pyplot.imread()   output 'RGB' format,and the cv2.imread() output the 'GBR' format.And the drive.py used Image.open() method which support 'RGB' format as default, when I modified this point, it's really get a better perfomance in simulator part than before.I used the numpy.filplr method to filp the image.
 

I finally randomly shuffled the data set and put 20% of the data into a validation set. 


I used this training data for training the model. The validation set helped determine if the model was over or under fitting. The ideal number of epochs was 7 as evidenced by the chart below,I tried many times,the overfitting would happened when epochs at 8 or 9,I used an adam optimizer so that manually training the learning rate wasn't necessary.
![alt text][image5]

I modified 2 positions in the drive.py:

(1).set the speed to 15; I think the steer is related to the speed ,and the speed is not in the training target,so need to finetune it,and the model should perform better if it match the mean speed in training mode.

(2).set the image size to 80x160,the former drive.py was designed for the original image size,so it need to resize the image as feeding data.

----------------------
**email**:*zooboy@vip.sina.com*
