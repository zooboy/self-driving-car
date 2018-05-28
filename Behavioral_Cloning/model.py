
# coding: utf-8

# In[1]:

import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt


# In[32]:

records =[]


# In[33]:

import os 
diclist = os.listdir("./records/")
# I catch and store the data  in this folder.


# In[34]:

csvlist =[]
for dic in diclist:
        subdiclist = os.listdir('./records/'+dic+'/')
        if('driving_log.csv'in subdiclist):
            logpath = './records/'+ dic+'/driving_log.csv'
            with open(logpath) as csvfile:
                reader= csv.reader(csvfile)
                csvlist.append(logpath)
                for line in reader :
                    records.append(line)
# load all of the 'driving_log' files in the folder.


# In[35]:

csvlist
#show the file list.


# In[36]:

len(records)
#show all lines in driving_log files.


# In[37]:

import re 
def getpath(pathlist):
    imagelist =[]
    for i in range(3):
        filepath = pathlist[i]
        diclist = re.split(r'[\\/]',filepath)
        filename = diclist[-1]
        dicname = 'data'
        if len(diclist)>3:
            dicname = diclist[-3]
        path = './records/'+dicname+'/IMG/'+filename
        img = plt.imread(path)
        imagelist.append(img)
    return imagelist

def showimage(imagelist,col,row,figsize):
    fig, axes = plt.subplots(col, row, sharex=True, sharey=True, figsize=figsize)
    for ii, ax in zip(range(3), axes.flatten()):
        img = imagelist[ii]
        ax.imshow(img, aspect='equal')
        ax.xaxis.set_visible(False)
        ax.yaxis.set_visible(False)
    plt.subplots_adjust(wspace=0, hspace=0)
    plt.show()
# function getpath() get the relative path of the image file 
# funtion showimage() show and check the image files.
# I had captured the images in training mode on the simulator in my two personal 
# computers(it is really difficult to record it, once I thought different GPU or CPU could make some effects on it ). 
# so I need to make image file source address from different computer and different path type(the sample data
# used relative path ) compatible.
# you can see this point also in the generator part.


# In[38]:

from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle


# In[39]:

correction = 0.2
lines= []
for i in range(len(records)):
    for j in range(3):
        path_value = []
        steer = float(records[i][3])
        if(j==1):
            steer = steer + correction
        elif(j==2):
            steer = steer -correction
        path_value.append(records[i][j])
        path_value.append(steer)
        lines.append(path_value)
lines = shuffle(lines)
#In this part,I split the lines to two parts: part 1: the 'src.' of image(3 sorts: the center,the left, the right)
#part 2 ,the steer value. I set the steer factor as 0.2. 
# left image steer value = center steer value+ 0.2,
# right image steer value =center steer value- 0.2,
# the output is a 2-dimensions Python List.first dimension store the image file address, the second one store the steer 
# value according the image.
# After that,I shuffle it.


# In[40]:

train_samples,val_samples =train_test_split(lines, test_size=0.2)
# I split the list to train set and the validation set,and set the validation ratio as 20%


# In[41]:


def generator (samples,batch_size =100):
    num_samples = len(samples)
    while 1:
        shuffle(samples)
        for offset in range(0,num_samples,batch_size):
            batch_samples = samples[offset:offset+batch_size]
            images = []
            angles =[]
            for batch_sample in batch_samples:
                source_path = batch_sample[0]
                measurement = float(batch_sample[1])
                diclist = re.split(r'[\\/]',source_path)
                filename = diclist[-1]
                subdic = 'data'
                if len(diclist)>3:
                    subdic = diclist[-3]
                current_path = './records/'+subdic+'/IMG/'+filename
                image = plt.imread(current_path)
                image = cv2.resize(image,(160,80))
                flipimage = np.fliplr(image)
                flipmeasurement = - measurement
                images.append(image)
                images.append(flipimage)
                angles.append(measurement)
                angles.append(flipmeasurement)
            X_train =np.array(images)
            y_train =np.array(angles)
            yield shuffle(X_train,y_train)
# consider the huge training and validation dataset would exhaust the memory,
# I define a generator to generate the data,batch by batch.
# beside this function, I read the image by pyplot.imread method,to get a 'RGB' format image array.
# and the cv2.imread() which showing in the class video output the image as format  'GBR',
# but the drive.py used Image.open() method which support 'RGB' format as default.
##############################################################################################################
# I resize the images and filp them, I resize them by half height and half width,as a result, 
# the image reserve a quarter pixels,it highly reduce the training time and make little influence to the accurate.
# I learn this from Paul Heraty's  'Behavioral Cloning Cheatsheet' which listed in the Slack forum.
################################################################################################################
# I flip the images to augment the dataset by numpy.fliplr() method.
# finally , shuffle and yield the X_train and y_train.


# In[42]:

source = getpath(records[60])
source_resize =[]
source_filp =[]
for i in range(3):
    img_resize =cv2.resize(source[i],(160,80))
    source_resize.append(img_resize)
    source_filp.append(np.fliplr(img_resize))
showimage(source,1,3,(30,5))
showimage(source_resize,1,3,(30,5))
showimage(source_filp,1,3,(30,5))
# the first row shows the original image.
# the second row shows the image that resized. It has lower resolution and looks a little blurred than the original one,
# but clear to see the lines,edges and the textrue of the lane
# the third row shows the image that resized and flipped.


# In[43]:

import math
batch_size = 32
train_generator = generator(train_samples,batch_size)
val_generator =generator(val_samples,batch_size)
train_steps = int(math.ceil(len(train_samples)/batch_size))
val_steps =int(math.ceil(len(val_samples)/batch_size))
# set the batch size
# my keras version need to set the train_steps/val_steps when training in 'fit_generator' mode. 
# the version is different from showing in class video.


# In[44]:

from keras.models import Sequential
from keras.layers import Dense,Flatten,Lambda,Conv2D,MaxPooling2D,Cropping2D,Dropout
from keras import optimizers


# In[45]:

from keras.callbacks import ModelCheckpoint
mcheckpoint_loss = ModelCheckpoint('model.h5', monitor='val_loss', verbose=0,
                                   save_best_only=True, save_weights_only=False, mode='auto', period=1)
# I add a callback function to monitor the lowest val-loss result and save it.


# In[46]:

model = Sequential()
model.add(Lambda(lambda x: x/255 - 0.5,input_shape =(80,160,3)))
model.add(Cropping2D(cropping = ((30,10),(0,0))))
model.add(Conv2D(24,(5,5),strides=(2,2),activation='relu'))
model.add(Conv2D(36,(3,3),strides=(2,2),activation='relu'))
model.add(Conv2D(48,(3,3),strides=(2,2),activation='relu'))
model.add(Conv2D(64,(2,2),strides=(1,1),activation='relu'))
model.add(Conv2D(64,(2,2),strides=(1,1),activation='relu'))

model.add(Flatten())
model.add(Dropout(0.4))
model.add(Dense(100,activation='relu'))
model.add(Dropout(0.4))
model.add(Dense(50,activation='relu'))
model.add(Dropout(0.4))
model.add(Dense(10,activation='relu'))
model.add(Dense(1))

model.compile(loss='mse',optimizer ='adam')
# I used a Nvidia model showing in the class video, I have tried several model ,
# the lenet,alexnet,finally I choosed this one.  It performed good.

# I used adam optimizer,so I don't need to set the learning rate manully.
# follow the class video,there is no activation function after the fully-connect layer.

# I made some modifies:
# 1.consider the image was resized as quarter as before, I reduce the kerel size from the second conv. layer.
# 2.add 3 dropout layers to fight with the overfitting. 


# In[47]:

history_object=model.fit_generator(train_generator,epochs=7,validation_data=val_generator,
                    steps_per_epoch=train_steps,validation_steps=val_steps,verbose=1,callbacks =[mcheckpoint_loss])
# I trained this model for 7 epochs.It looks not overfitting.
# In my experence this model would be overfitting when the training epochs over 8 times.


# In[19]:

print(history_object.history.keys())


# In[20]:

plt.plot(history_object.history['loss'])
plt.plot(history_object.history['val_loss'])
plt.title('model mean squared error loss')
plt.ylabel('mean squared error loss')
plt.xlabel('epoch')
plt.legend(['training set', 'validation set'], loc='upper right')
plt.show()
#show the training loss VS. validation loss, it looks no overfitting yet.the training loss is closer to validation loss,and 
# the validation loss seems reach the plateau state.


# In[ ]:

get_ipython().magic('run drive.py model.h5 run1')
# run the simulator.


# In[24]:

get_ipython().magic('run video.py run1')
# record the video with fps 60 .


# In[ ]:



