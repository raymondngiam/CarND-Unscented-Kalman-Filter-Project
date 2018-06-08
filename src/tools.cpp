#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {
	f_nis_lidar.open("nis_lidar.csv", std::ios_base::out);
	f_nis_lidar<<"nis_lidar"<<std::endl;
	f_nis_radar.open("nis_radar.csv", std::ios_base::out);
	f_nis_radar<<"nis_radar"<<std::endl;
	f_predictions.open("predictions.csv", std::ios_base::out);
	f_predictions<<"x,y,vx,vy,x_gt,y_gt,vx_gt,vy_gt"<<std::endl;
}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//	* the estimation vector size should not be zero
	//	* the estimation vector size should equal ground truth vector size
	if(estimations.size() != ground_truth.size() || estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for(unsigned int i = 0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array() * residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse / estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

void Tools::LogNIS(MeasurementPackage meas_package, double nis){
	//Push the NIS 
	if (meas_package.sensor_type_ == MeasurementPackage::RADAR)
		f_nis_radar<<nis<<std::endl;
	if (meas_package.sensor_type_ == MeasurementPackage::LASER)
		f_nis_lidar<<nis<<std::endl;
}

void Tools::LogPredictions(double x,double y, double vx, double vy,
double x_gt,double y_gt, double vx_gt, double vy_gt){
	f_predictions<<x<<","<<y<<","<<vx<<","<<vy<<","
		<<x_gt<<","<<y_gt<<","<<vx_gt<<","<<vy_gt<<std::endl;

}

