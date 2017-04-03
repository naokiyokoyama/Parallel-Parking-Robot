const float kp = 5.0;
const float ki = 0.0000;
const float kd = 1.1;
const float minI =  -20.0 / ki;
const float maxI = 20.0 / ki;

float P,I,D,I_tot;

int PIDcompute(float error, float error_rate) {
  P = kp*error;
  I_tot += error;
  if(I_tot < minI)
    I_tot = minI;
  else if(I_tot > maxI)
    I_tot = maxI;
  I = ki*I_tot;
  D = kd*error_rate;
  return P + I + D;
}
