# **Semantic Segmentation Project** 




[//]: # (Image References)
[image1]: ./loss.png "loss statics"
[image2]: ./final_loss.png "loss print"
[image3]: ./results.png "samples"

### The project information and code running dependencies link : [project info](https://github.com/udacity/CarND-Semantic-Segmentation) 

#### 1.Overview
The project  used a FCN-VGG model to generate semantic-segmentation images.By FCN model we can label each pixel of the original images to a specific class.The FCN model can be divided to 2 parts : the encoder and the decoder.In encoder part, it   works just like a conventional Convnet architecture:conv layers and pooling layers, in this part , mainly used some pretrained architecture,like vgg ,googleNet and so on, but instead the final fully connected layer in these classic architecture to 1x1 fully convolution layer, the spatial information was recorded, That is why we call it Fully Convolution Network(FCN) .The decoder part is a de-convolution process.It uses Skip Connection to combine the output of two layers which can enhance the classify precision observably, and then ,the de-convolution process upsample the previous layer to a higher resolution or dimension. So, after that, each pixel become a 'x',and the 'y' is the class labels. 


#### 2.Code Frame
I complete the code basically according the paper **Fully Convolutional Networks for Semantic Segmentation**.I add a `REGULARIZATION_LOSSES` to the loss, I think it is very important to add it, this item provided a  quite visible improvment to the result.Within this item,  the inference results much smoother on the edge pixels. 
I add a return value in function `train_nn` type as `dataframe` to show the mean loss statics graph, like below :




 ![alt text][image1]


#### 3.Hyperparameters Choosen
I train the model in my local notebook(CPU:i7,internal memory:16G; GPU: GTX 1070,graphics memory:8G), I tried several  hyperparameters combos, finally I set the epoches as 25 ,and the batch size like 5, the learning rate as 1e-4 ,the keep prob as 0.4 , the final mean loss was stable around 0.042.
 ![alt text][image2]


#### 4.The Final Result


Basically speaking, in most cases, the model can correctly identify the free space in these roads in these images.But there is still some error classification in these images,especially in the borderline of roads. 

The inference result of these samples like below:
 ![alt text][image3]




