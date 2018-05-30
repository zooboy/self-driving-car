from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import datetime
import cv2
import h5py

class TLClassifier(object):
    def __init__(self,sim):
        """

        """
        self.sim = sim
        self.model = None
        self.threshold = 0.5
        # I use two kinds of method to detect the red lights :
        # In simulator mode ,I use open cv method
        # In real world images detection, I use a CNN model based on keras package
        # for the CNN model detail you can check the file 'traffic_lights_detect_training.ipynb' folder in \light_classification


        if self.sim==False:
            from keras.models import load_model
            model_dir = r'light_classification/tl_7.h5'
            self.model = load_model(model_dir)





    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if self.sim :
            state, show_img = self.cv_detect(image)
        else:
            state,show_img = self.CNN_detect(image)

        return state,show_img


    def CNN_detect(self,image):
      # If the simulator mode is off , use a CNN model to detect the image.
        if self.model is not None:
            #start = datetime.datetime.now()
            state = TrafficLight.UNKNOWN
            show_img = image.copy()
            img = cv2.resize(image,(80,60))
            img = np.array([img])
            classes = self.model.predict_classes(img,verbose=0)
            prob = np.max(self.model.predict(img))

            #end = datetime.datetime.now()
            #c = end - start
            #print('inference duration (sec):',c.total_seconds())
            #print('class:',classes.shape)
            #print('prob:',prob.shape)
            if classes == 0 and prob >= self.threshold:
                state = TrafficLight.RED
                #print('RED,CNN_detect')
            #else:
                #print('Not RED,CNN_detect')
            return state,show_img

    def cv_detect(self,image) :
        # If the simulator mode is on, use a OpenCV method to detect the red lights in images that camara had caught.

        #start = datetime.datetime.now()
        state = TrafficLight.UNKNOWN
        show_img = image.copy()
        # Transform the image to HSV color space.Compare to other color spaces,HSV is considered
        # as a user oriented color space, in other words, the HSV space feel the color more likely to  human eyes
        # and easy to define color by assigning H value.
        HSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Then  need to define a red color recognize band.
        lower_band = np.array([0,50,50])
        upper_band = np.array([10,255,255])
        red = cv2.inRange(HSV, lower_band , upper_band)

        # here add a  complementary color recognize band.because there will cause a complementary cyan color(phase difference
        # 180 degree) at the edge of red color adjoin to black color
        #
        lower_band = np.array([170,50,50])
        upper_band = np.array([180,255,255])
        complementary = cv2.inRange(HSV, lower_band , upper_band)

        # overlap these 2 color layers at all,that would make the circle more plump,
        # then filter the noise dots.
        overlap = cv2.addWeighted(red, 1.0, complementary, 1.0, 0.0)
        noise_remove = cv2.medianBlur(overlap,7)
        # add some Gaussian Blur to smooth the edge.
        blur = cv2.GaussianBlur(noise_remove,(15,15),0)

        # detect the circles.
        circles = cv2.HoughCircles(blur,cv2.HOUGH_GRADIENT,0.5,41, param1=70,param2=30,minRadius=5,maxRadius=150)
        #end = datetime.datetime.now()
        #c = end - start
        #print('inference duration (sec):',c.total_seconds())

        if circles is not None:
            state = TrafficLight.RED
            #print("RED,CV2")
        #else:
            #print("Not RED,CV2")


        return state, show_img
