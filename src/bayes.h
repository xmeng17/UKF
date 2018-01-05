#ifndef BAYES
#define BAYES

#include "motion.h"
#include "sensor.h"
#include "measurement.h"

class Bayes {
public: 
  Measurement prev_meas_;
  Measurement meas_;
  Motion* motion;
  Sensor* sensor;

  Bayes(){};
  virtual ~Bayes(){
    delete motion;
    delete sensor;
  };

  //returns nis
  virtual double Fuse(Measurement meas) = 0;
  virtual void Prediction() = 0;
  virtual void Init() = 0;
  virtual double Correction() = 0;
};

#endif
