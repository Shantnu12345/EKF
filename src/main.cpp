#include <iostream>
#include <math.h>
#include <fstream>
#include <random>
#include "FusionEKF.h"
#include "tools.h"
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;

//Tune the two cases

void plotData(const vector<VectorXd> &measurements,
							const vector<VectorXd> &estimations,
							const vector<VectorXd> &ground_truth,
							float rmse, 
							int test)

{
	int n=measurements.size();
	vector<double> xAxis(n);
	for(int i=0; i<n; i++) xAxis[i]=i;

	vector<double> meas, est, gt;
	for(VectorXd vec:measurements) meas.push_back(vec(0));
	for(VectorXd vec:estimations)  est.push_back(vec(0));
	for(VectorXd vec:ground_truth) gt.push_back(vec(0));

	string m = test==1 ? "cam_data1_noisy" : "cam_data2";
	string g = test==1 ? "cam_data1 (GroundTruth)" : "rad_data2 (GroundTruth)";
	plt::named_plot(m, xAxis, meas);
	plt::named_plot("Estimations", xAxis, est);
	plt::named_plot(g, xAxis, gt);
	string s = "Testcase:" + to_string(test) + " RMSE:" + to_string(rmse);
	plt::title(s);
  
	plt::legend();
	//plt::axis("equal");
	plt::show();
}

int main()
{
	Tools tools;

	{
	cout<<"=============================================TEST CASE 1==============================================================="<<endl;
  FusionEKF fusionEKF;
	fusionEKF.sigma_acceleration = 3.0f;
	fusionEKF.sigma_sensor       = 0.5f;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;
	vector<VectorXd> noisy_measurements;
	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0.0,fusionEKF.sigma_sensor);

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
	
	VectorXd RMSE = tools.CalculateRMSE(estimations, ground_truth);  	 
	cout<<"Test case 1: RMSE:"<<RMSE<<endl;
	
	plotData(noisy_measurements,estimations, ground_truth, RMSE(0), 1);
	}
	
	{
	cout<<"=============================================TEST CASE 2==============================================================="<<endl;
	FusionEKF fusionEKF2;
	fusionEKF2.sigma_acceleration = 3.0f;
	fusionEKF2.sigma_sensor       = 10.5f;
  vector<VectorXd> estimations2;
  vector<VectorXd> ground_truth2;
	vector<MeasurementPackage> radardata;
	vector<VectorXd> cameraData2;
	string temp;
	
	ifstream gtfile;
	gtfile.open("../data/rad_data2.txt");
	if (!gtfile.is_open())
	{
		throw runtime_error(" GT File couldn't be opened");
	}

	while(getline(gtfile, temp))
	{
		MeasurementPackage measurement;
		int pos = temp.find(",");
		measurement.timestamp_ = stol(temp.substr(0, pos));
		double meas = stod(temp.substr(pos+1));
		measurement.raw_measurements_ = VectorXd(MEASUREMENT_SIZE);
		measurement.raw_measurements_ << meas;
		radardata.push_back(measurement);
	}

	ifstream infile2;
	infile2.open("../data/cam_data2.txt");
	if (!infile2.is_open())
	{
		throw runtime_error("File couldn't be opened");
	}

	while(getline(infile2, temp))
	{
		MeasurementPackage measurement;
		int pos = temp.find(",");

		measurement.timestamp_ = stol(temp.substr(0, pos));
		
		double meas = stod(temp.substr(pos+1));
		cout<<endl<<"New Measurement : " << meas<<" ============="<<endl;
		measurement.raw_measurements_ = VectorXd(MEASUREMENT_SIZE);
		measurement.raw_measurements_ << meas;
		cameraData2.push_back(measurement.raw_measurements_);

		VectorXd gt(MEASUREMENT_SIZE);
		gt(0)=tools.computeLastRadarMeasurement(radardata, measurement.timestamp_);
		ground_truth2.push_back(gt);

		fusionEKF2.ProcessMeasurement(measurement);  

		VectorXd estimate(MEASUREMENT_SIZE);		
		estimate(0) = fusionEKF2.ekf_.x_(0);		
		estimations2.push_back(estimate);
	} 
	
	VectorXd RMSE2 = tools.CalculateRMSE(estimations2, ground_truth2);  	 
	cout<<"Test case 2 - RMSE:"<<RMSE2<<endl;
	
	plotData(cameraData2, estimations2, ground_truth2, RMSE2(0), 2);
	
	}
}























































































