#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  //Compare the NIS result, I modify this value to 1.5.
  std_a_ = 1.5;
  

  // Process noise standard deviation yaw acceleration in rad/s^2
  //Compare the NIS result, I modify this value to 0.5.
  std_yawdd_ =0.5;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.

  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
 if (!is_initialized_) {
	 /* at the first time I initialize  x_,P_,n_x,n_aug,lambda,Rcount_,Lcount_,Rnis,Lnis,
	 the shape of Xsig_pred_ and weights_ */ 
	  cout << "The UKF Start: " << endl;
     x_ << 1, 1, 1, 1,1;

	P_ << 1, 0, 0, 0, 0,
	      0, 1, 0, 0, 0,
		  0, 0, 1, 0, 0,
		  0, 0, 0, 1, 0,
		  0, 0, 0, 0, 1; 

	n_x_ =5;
	n_aug_ = 7 ;
    lambda_ = 3 - n_aug_;
	Rcount_ = 0.0;
	Lcount_ = 0.0;
	Rnis_ = 0.0;
	Lnis_ = 0.0;

    Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

    weights_ = VectorXd(2*n_aug_+1);


	  for(int i = 0 ;i < weights_.size();i++){
	  if(i==0){
		  weights_(i) = lambda_/(lambda_+i+n_aug_);
	  }
	  else{
		  weights_(i) = 0.5/(lambda_+n_aug_);
	  }
  }
	 if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		 /* if the first received data is from radar sensor,
		 1.the Rcount_ counting.
		 2.get the rho,phi,rho_dot value to calculate the px,py,v,yaw(phi),yawrate(phidot),and I think the yawrate can not be read 
		   directly,so I set it to 0.
		 3.record the timestamp . 
		 
		 */
		 Rcount_+=1;
		float rho = meas_package.raw_measurements_[0];
		float phi =  meas_package.raw_measurements_[1];
		float rod =  meas_package.raw_measurements_[2];

		float rpy = sin(phi)*rho;
		float rpx = cos(phi)*rho;
		float rv = rho*rod;
		float phidot = 0;
		x_<< rpx,rpy,rv,phi,phidot;
		time_us_ = meas_package.timestamp_;
	 }
	 else  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
		 /* if the first received data is from lidar sensor,
		 1.the Lcount_ + 1  
		 2.allocate the first two value of the sensor received to  px,py,and for the first time ,the other values can not be read 
		   directly,so I set them to 0.

		 */
		 Lcount_+=1;
		 x_ << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], 0, 0, 0;
		time_us_ = meas_package.timestamp_;
	 }

	is_initialized_ = true;

    return;

 }

   	double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
	time_us_ = meas_package.timestamp_;
	
	Prediction(dt);
	
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		Rcount_+=1;
		
		UpdateRadar(meas_package);
		
	}
	else{
		Lcount_+=1;
		
		UpdateLidar(meas_package);
		
	}
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

 /* firstly, generate the augmented sigma points.*/

 //create augmented state vector
  VectorXd x_aug = VectorXd(7);
  //create augmented state covariance matrix
  MatrixXd P_aug = MatrixXd(7, 7);
  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  //create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_*std_a_;
  P_aug(6,6) = std_yawdd_*std_yawdd_;


  //create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  //create augmented sigma points
  Xsig_aug.col(0)  = x_aug;
  for (int i = 0; i< n_aug_; i++)
  {
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
  }

  /* secondly, create predict sigma points */
  
   for (int i = 0; i< 2*n_aug_+1; i++)
  {
    //extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

	
    //predicted state values,
	//read and calculate the predict value by following the motion equation
    double px_p, py_p;

    //avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
        py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }
    else {
        px_p = p_x + v*delta_t*cos(yaw);
        py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;
    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
	
  }
  
/* thirdly, inverse the process of sigma points generating, to generate the predict x and P */
  x_.fill(0.0);
  for(int k = 0;k<Xsig_pred_.cols();k++){
	  x_ =x_+ weights_(k)*Xsig_pred_.col(k); 
  }
  P_.fill(0.0);
  for(int j = 0;j<Xsig_pred_.cols();j++){
	  VectorXd x_diff = Xsig_pred_.col(j) - x_;
    //angle normalization
    while (x_diff(3)>M_PI )
		x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) 
		x_diff(3)+=2.*M_PI;
	P_ = P_+  weights_(j) * x_diff * x_diff.transpose() ;
  }





}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
	/*
	In lidar mode, the lidar only measured two values, the px and py,
	the 'H ' function is a matrix as: 
	1,0,0,0,0
	0,1,0,0,0
    the measurement process is a linear equation,that means we can calculte the update value 
	following the classic Kalman Filter method.  

	*/

	
	VectorXd z = VectorXd(2);
	MatrixXd H_ = MatrixXd(2,5);
	H_.fill(0.0);
	H_(0,0) = 1;
	H_(1,1) = 1;
	z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
	
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd R_ = MatrixXd(2,2);
	R_ << std_laspx_*std_laspx_,0,
		  0,std_laspy_*std_laspy_;
	

	// calculate the Kalman gain.
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;
	
	
	//calculate the new estimate of  X and P .
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
	// calculate the NIS value.
	float nis = (z - z_pred).transpose()*Si*(z - z_pred);
    // the 2-dimensions chi-square value (X2~0.95) is 5.991  
	float chi_value = 5.991;
	//  if the NIS value lower than chi square value, variable  Lnis ++;   
	if(nis < chi_value){
	  Lnis_+=1;
  }
 // calculate and print the NIS value/ratio.
  float chi_rate = Lnis_/Lcount_;
  cout <<"the Lidar NIS is :"<<nis<<" the ratio of lower than 5.991 is : "<<chi_rate<<endl; 
}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

  /* the radar measurement is not a linear process, so ,it need to transform the predicted sigma points to radar measurment space,
   then calculate the final predict vector Z-pred  and measurement covariance matrix S based the weights of each  sigma points of Z predicted matrix.
  */
  int n_z = 3;
  lambda_ = 3 - n_aug_;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);

  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);

	  Zsig.fill(0.0);
// transform the sigma points of predicted state X to measurment space.  
  for(int i = 0;i < Xsig_pred_.cols();i++){
	 double px = Xsig_pred_(0,i);
	 double py = Xsig_pred_(1,i);
	 double pv =  Xsig_pred_(2,i);
	 double phi = Xsig_pred_(3,i);

	 double rho = sqrt(px*px + py*py);
	 double	hfi = atan2(py,px);
	 double hfd = (px*cos(phi)*pv+py*sin(phi)*pv)/rho;

	 Zsig(0,i) = rho;
	 Zsig(1,i) = hfi;
	 Zsig(2,i) = hfd;
  }
  z_pred.fill(0.0);
  for(int k = 0; k< Zsig.cols();k++){

	  z_pred = z_pred + weights_(k)*Zsig.col(k);
  }
  S.fill(0.0);
  // calculate the S matrix,normalize the angle to [- pi,pi]  
  for(int j = 0;j<Zsig.cols();j++){

	VectorXd z_diff = Zsig.col(j) - z_pred;
    while (z_diff(1)>M_PI ) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
	S = S + weights_(j) * z_diff * z_diff.transpose() ;
  }
  
 //define the measurment noise matrix R

   MatrixXd R = MatrixXd(n_z,n_z);



     R <<    std_radr_*std_radr_, 0, 0,
          0, std_radphi_*std_radphi_, 0,
          0, 0,std_radrd_*std_radrd_;
     S = S + R;

// read the measured data from radar sensor
	float rho = meas_package.raw_measurements_[0];
	float phi =  meas_package.raw_measurements_[1];
	float rod =  meas_package.raw_measurements_[2];
	VectorXd z = VectorXd(n_z);

	z <<rho,phi,rod;


  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);


  //update state mean and covariance matrix
  Tc.fill(0.0);
  for(int i = 0;i<Xsig_pred_.cols();i++){
	  VectorXd x_diff = Xsig_pred_.col(i) - x_;
	  VectorXd z_diff = Zsig.col(i) - z_pred;
	  while (x_diff(3)>M_PI ) x_diff(3)-=2.*M_PI;
	  while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;
	  while (z_diff(1)>M_PI ) z_diff(1)-=2.*M_PI;
      while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
	  Tc = Tc + weights_(i)*x_diff*z_diff.transpose();
  }
  // calculate the Kalman Gain.
  MatrixXd Kk = Tc*S.inverse();
  x_ = x_ + Kk*(z-z_pred);
  P_ = P_ -Kk*S*Kk.transpose();
  // calculate the NIS value,print the ratio information.
  float nis = (z - z_pred).transpose()*S.inverse()*(z - z_pred);
  float chi_value = 7.815;
  if(nis < chi_value){
	  Rnis_+=1;
  }
  float chi_rate = Rnis_/Rcount_;
  cout <<"the Radar NIS is :"<<nis<<" the ratio of lower than 7.815 is : "<<chi_rate<<endl; 

}
