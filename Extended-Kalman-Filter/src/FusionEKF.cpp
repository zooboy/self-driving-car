#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix initialize
  // The R_radar_,R_laser_ and H_laser were constant matrix that not be modified in this project Kalman Filter workflow. 
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  H_laser_ << 1, 0, 0, 0,
			  0, 1, 0, 0;


  // The Hj_ matrix was calculated  every time based on the predict 'X' . So I don't think the Hj_ need to initialize


  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
	// initialize the X ,F and P ,in this project the F not need to replace by Fj when radar predict mode.
	// for the first frame,the X was renew based on the type of sensor data feedback.
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
	ekf_.F_ = MatrixXd(4, 4);
	ekf_.F_ << 1, 0, 1, 0,
			  0, 1, 0, 1,
			  0, 0, 1, 0,
			  0, 0, 0, 1;
	ekf_.P_ = MatrixXd(4, 4);
	ekf_.P_ << 1, 0, 0, 0,
			  0, 1, 0, 0,
			  0, 0, 500, 0,
			  0, 0, 0, 500;
	


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
		// if the first frame was radar data, it should be convert to cartesian from polar .
		// in radar mode,for the first frame, it can conclude all the state vector: px,py,vx,vy
		float rho = measurement_pack.raw_measurements_[0];
		float phi =  measurement_pack.raw_measurements_[1];
		float rod =  measurement_pack.raw_measurements_[2];
		float rpy = sin(phi)*rho;
		float rpx = cos(phi)*rho;
		float rvx = cos(phi)*rho*rod;
		float rvy = sin(phi)*rho*rod;
		ekf_.x_<< rpx,rpy,rvx,rvy;
		previous_timestamp_ = measurement_pack.timestamp_;

		//cout << "EKF: RADAR INI" << endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
		// if the first frame was lidar data, just put the px,py into X, the vx, vy  were not clear in this moment,so set it to 0,0.
		ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
		previous_timestamp_ = measurement_pack.timestamp_;

		//cout << "EKF: LASER INI" << endl;

    }
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  // update the F matrix based on the delta-t past.
  	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;
	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;

	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;
	float	noise_ax = 9;
    float	noise_ay = 9;

	// udapte  process noise covariance matrix Q.
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
			   0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
			   dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
			   0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

	ekf_.Predict();
	

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
	  
	  // calculate the Hj matrix,then update the px,py,vx,py using extended kalman filter update equation if the senser type was radar.
	  Hj_ =  tools.CalculateJacobian(ekf_.x_);
	  ekf_.R_ = R_radar_;
	  ekf_.H_ = Hj_;
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    // Radar updates
  } 
  else {
	  ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;
	  ekf_.Update(measurement_pack.raw_measurements_);
    // Laser updates
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
