#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"
#include <fstream>
#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

	// used to log the NIS
	ofstream f_nis_lidar;
	ofstream f_nis_radar;

	ofstream f_predictions;

	void LogNIS(MeasurementPackage meas_package, double nis);

	void LogPredictions(double x,double y, double vx, double vy,
		double x_gt,double y_gt, double vx_gt, double vy_gt);
};

#endif /* TOOLS_H_ */
