#include <iostream>
#include <math.h>
#include <fstream>
#include <random>
#include "FusionEKF.h"
#include "tools.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;


void plotData(const vector<VectorXd> &measurements,
							const vector<VectorXd> &estimations,
							const vector<VectorXd> &ground_truth)

{
	int n=measurements.size();
	vector<double> xAxis(n);
	for(int i=0; i<n; i++) xAxis[i]=i;

	vector<double> meas, est, gt;
	for(VectorXd vec:measurements) meas.push_back(vec(0));
	for(VectorXd vec:estimations)  est.push_back(vec(0));
	for(VectorXd vec:ground_truth) gt.push_back(vec(0));

	plt::named_plot("Measurements", xAxis, meas);
	plt::named_plot("Estimations", xAxis, est);
	plt::named_plot("GroundTruth", xAxis, gt);
  
	plt::legend();
	//plt::axis("equal");
	plt::show();
}

int main()
{

  // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
	vector<VectorXd> noisy_measurements;
	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0.0,0.5);

	ifstream infile;
	infile.open("../data/cam_data1.txt");
	string temp;
	if (!infile.is_open())
	{
		throw runtime_error("File couldn't be opened");
	}

	while(getline(infile, temp))
	{
		MeasurementPackage measurement;
		int pos = temp.find(",");

		measurement.timestamp_ = stol(temp.substr(0, pos));
		
		double meas = stod(temp.substr(pos+1));
		cout<<endl<<"New Measurement : " << meas<<" ============="<<endl;

		VectorXd gt(MEASUREMENT_SIZE);
		gt(0)=meas;
		ground_truth.push_back(gt);
		
		double noise = distribution(generator);
		VectorXd noisyMeasurement(MEASUREMENT_SIZE);
		noisyMeasurement << meas+noise;
		noisy_measurements.push_back(noisyMeasurement);

		cout<<"Noisy Measurement : " << meas+noise<<endl;
		measurement.raw_measurements_ = noisyMeasurement;

		fusionEKF.ProcessMeasurement(measurement);  

		VectorXd estimate(MEASUREMENT_SIZE);		
		estimate(0) = fusionEKF.ekf_.x_(0);		
		estimations.push_back(estimate);
	} 
	
	//VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);  	 
	
	plotData(noisy_measurements,estimations, ground_truth);
	
}























































































