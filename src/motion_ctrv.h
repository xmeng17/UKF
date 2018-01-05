#ifndef MOTION_CTRV
#define MOTION_CTRV

#include "motion.h"
#include <cmath>

#define STD_A 1.5
#define STD_ALPHA 1.0

using namespace std;

class CTRV : public Motion {
public:

  CTRV();
  virtual ~CTRV();

  VectorXd GetMotion(VectorXd aug_x, double dt);
  VectorXd GetNoise(VectorXd aug_x, double dt);
  VectorXd Predict(VectorXd aug_x, double dt);

};

#endif
