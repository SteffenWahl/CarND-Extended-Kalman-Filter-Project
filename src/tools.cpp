#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

using namespace std;
Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  if(estimations.size() != ground_truth.size())
  {
    cout << "estimations.size() != ground_truth.size()" << endl;
    return rmse;
  }
  if(estimations.size() == 0)
  {
    cout << "estimations.size() == 0" << endl;
    return rmse;
  }

  for(unsigned int i = 0; i < estimations.size(); ++i)
  {
    VectorXd tmp = estimations[i]-ground_truth[i];
    tmp = tmp.array()*tmp.array();
    rmse += tmp;
  }
  rmse = rmse/estimations.size();
  return rmse.array().sqrt();
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float c1 = px*px+py*py;
  float c2 = sqrt(c1);
  float c3 = c1*c2;

  if( fabs(c1) < 0.0001)
  {
    cout << "fabs(c1) < 0.0001" << endl;
    return Hj;
  }

  Hj << (px/c2), (py/c2), 0, 0,
       -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;
  return Hj;

}


MatrixXd Tools::F(float dt)
{
  MatrixXd F(4,4);
  F << 1,0,dt,0,
       0,1,0,dt,
       0,0,1,0,
       0,0,0,1;
  return F;
}

MatrixXd Tools::Q(float dt)
{
  MatrixXd Q(4,4);
  float nax = 9;
  float nay = 9;

  float nax2 = nax*nax;
  float nay2 = nay*nay;

  float dt2 = dt*dt;
  float dt3 = dt*dt2;
  float dt4 = dt2*dt2;

  Q << dt4/4.*nax2,           0, dt3/2.*nax2,           0,
                 0, dt4/4.*nay2,           0, dt3/2.*nay2,
       dt3/2.*nax2,           0,    dt2*nax2,           0,
                 0, dt3/2.*nay2,           0,    dt2*nay2;
  return Q;
}

