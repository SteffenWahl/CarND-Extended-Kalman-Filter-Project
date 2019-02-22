#include "measurement_package.h"

#include <iostream>

using namespace std;

ostream& operator<<(ostream& os, const MeasurementPackage& mp)
{
  os << "MeasurementPackage Time:" << mp.timestamp_;
  if( mp.sensor_type_ == MeasurementPackage::LASER)
  {
    os << " LASER" << endl;
  } else if (mp.sensor_type_ == MeasurementPackage::RADAR)
  {
    os << " RADAR" << endl;
  } else
  {
    os << " UNKNOWN" << endl;
  }
  os << mp.raw_measurements_ << endl;
  return os;
}
