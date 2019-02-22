#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

#include <iostream>

class MeasurementPackage {
 public:
  enum SensorType{
    LASER,
    RADAR
  } sensor_type_;

  long long timestamp_;

  Eigen::VectorXd raw_measurements_;

  friend std::ostream& operator<<(std::ostream& os, const MeasurementPackage& mp);
};
#endif // MEASUREMENT_PACKAGE_H_
