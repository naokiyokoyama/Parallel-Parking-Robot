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
    P = kp*error;
    I = ki*I_tot;
    D = kd*error_rate;
    return P + I + D;
  }
  int compute(float error) {
    static unsigned long prev,curr;
    static float eprev;
    prev = curr;
    curr = micros();
    float D_term = (error-eprev)/((float)(curr-prev));
    I_tot += error;
    I_tot = constrain(I_tot,minI,maxI);
    P = kp*error;
    I = ki*I_tot;
    D = kd*D_term;
    eprev = error;
    return P + I + D;
  }

  float kp,ki,kd,minI,maxI,I_tot,P,I,D; 
};

const float kp1 = 5.0;
const float ki1 = 0.0000;
const float kd1 = 1.1;
const float minI1 =  -25.0 / ki1;
const float maxI1 = 25.0 / ki1;
PID imuDrive(kp1,ki1,kd1,minI1,maxI1);

const float kp2 = 0.8*1000000; //data3 = 2
const float ki2 = 210000;
//const float kp2 = 1.3*1000000; //data3 = 2
//const float ki2 = 220000;
const float kd2 = 0;//6*10000000000; //.02
const float minI2 =  -200.0/ki2;
const float maxI2 = 200.0/ki2;
PID encL(kp2,ki2,kd2,minI2,maxI2);
const float kp3 = 0.9*1000000; //data3 = 2
const float ki3 = 230000;
const float kd3 = 0;//6*10000000000; //.02
const float minI3 =  -200.0/ki3;
const float maxI3 = 200.0/ki3;
PID encR(kp3,ki3,kd3,minI3,maxI3);
