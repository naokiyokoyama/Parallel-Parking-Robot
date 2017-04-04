class PID {
  public:
  PID(float Kp,float Ki,float Kd,float MinI,float MaxI) {
    kp = Kp;
    ki = Ki;
    kd = Kd;
    minI = MinI;
    maxI = MaxI;
  }
  int compute(float error,float error_rate) {
    I_tot += error;
    if(I_tot < minI)
      I_tot = minI;
    else if(I_tot > maxI)
      I_tot = maxI;
    float P = kp*error;
    float I = ki*I_tot;
    float D = kd*error_rate;
    return P + I + D;
  }
  int compute(float error) {
    static unsigned long prev,curr;
    static float eprev;
    prev = curr;
    curr = micros();
    float D_term = (error-eprev)/((float)(curr-prev));
    I_tot += error;
    if(I_tot < minI)
      I_tot = minI;
    else if(I_tot > maxI)
      I_tot = maxI;
    float P = kp*error;
    float I = ki*I_tot;
    float D = kd*D_term;
    eprev = error;
    return P + I + D;
  }

  float kp,ki,kd,minI,maxI,I_tot; 
};

const float kp1 = 5.0;
const float ki1 = 0.0000;
const float kd1 = 1.1;
const float minI1 =  -20.0 / ki1;
const float maxI1 = 20.0 / ki1;
PID imuDrive(kp1,ki1,kd1,minI1,maxI1);

const float kp2 = 0.006;
const float ki2 = 0.0000;
const float kd2 = 0.0;
const float minI2 =  -20.0 / ki1;
const float maxI2 = 20.0 / ki1;
PID encDrive(kp2,ki2,kd2,minI2,maxI2);

