#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  //measurement covariance matrix - laser
  ekf_.R_ = MatrixXd(1, 1);
  ekf_.R_ << sigma_sensor*sigma_sensor;

  ekf_.H_ = MatrixXd(1, 2);
  ekf_.H_ << 1,0;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

/*
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    // first measurement
    ekf_.x_ = VectorXd(2);
    ekf_.x_ << measurement_pack.raw_measurements_[0], 0;

    ekf_.F_ = MatrixXd(2, 2);
    ekf_.F_ << 1, 1,
               0, 1;

    // state covariance matrix
    ekf_.P_ = MatrixXd(2, 2);
    ekf_.P_ << 1, 0,
               0, 1;

    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  }

  //  cout<<"second measuremnet"<<endl;
	double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000.0f;	//dt - expressed in seconds
	previous_timestamp_ = measurement_pack.timestamp_;
	

	double dt_2 = dt * dt;
	double dt_3 = dt_2 * dt;
	double dt_4 = dt_3 * dt;
  double variance_a=sigma_acceleration*sigma_acceleration;
  cout<<"dt:"<<dt<<endl;
	//Modify the F matrix so that the time is integrated
	ekf_.F_(0, 1) = dt;

	//set the process covariance matrix Q - Derived on wikipedia
	ekf_.Q_ = MatrixXd(2, 2);
	ekf_.Q_ <<  (dt_4/4)*variance_a  , (dt_3/2)*variance_a,
			        (dt_3/2)*variance_a  , dt_2*variance_a;
  ekf_.Predict();
  
  cout << "   Prediction:"<<endl<< "     x_ = (" << ekf_.x_(0)<< ","<<ekf_.x_(1)<<")"<< endl;
  cout << "     P_ = "<<ekf_.P_(0) << " "<< ekf_.P_(1) <<endl<<"          "<<ekf_.P_(2) << " "<< ekf_.P_(3)<<endl; 
  //cout << ekf_.P_ << endl;

  ekf_.Update(measurement_pack.raw_measurements_);
  cout << "   Update:"<<endl<< "     x_ = (" << ekf_.x_(0)<< ","<<ekf_.x_(1)<<")"<< endl;
  cout << "     P_ = "<<ekf_.P_(0) << " "<< ekf_.P_(1) <<endl<<"          "<<ekf_.P_(2) << " "<< ekf_.P_(3)<<endl; 

}
