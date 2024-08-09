//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, pwm_L_F, pwm_L_B, pwm_R_F, pwm_R_B, roll_error, roll_previous_error;
float roll_pid_p = 0;
float roll_pid_i = 0;
float roll_pid_d = 0;
///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp = 0.7;
double roll_ki = 0.006;
double roll_kd = 1.2;
float roll_desired_angle = 0;

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p = 0;
float pitch_pid_i = 0;
float pitch_pid_d = 0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp = 0.72;
double pitch_ki = 0.006;
double pitch_kd = 1.22;
float pitch_desired_angle = 0;

//////////////////////////////PID FOR YAW//////////////////////////
float yaw_PID, yaw_error, yaw_previous_error;
float yaw_pid_p = 0;
float yaw_pid_i = 0;
float yaw_pid_d = 0;
///////////////////////////////YAW PID CONSTANTS///////////////////
double yaw_kp = 0.75;
double yaw_ki = 0.005;
double yaw_kd = 1.0;
float yaw_desired_angle = 0;

//////////////////////////////GENERAL PID CONSTANTS///////////////////
float elapsedTime, time, timePrev;
float desireMapAngle = 90;
float PIDContraint = 500;

class pid {
  private:
    #define minPWM 1000
    #define maxPWM 1900

  public:
    void MapAngles(int ROLL, int PITCH, int YAW) {
      roll_desired_angle = map(ROLL, 1000, 2000, -desireMapAngle, desireMapAngle);
      pitch_desired_angle = map(PITCH, 1000, 2000, -desireMapAngle+1, desireMapAngle-1);
      yaw_desired_angle = map(YAW, 1000, 2000, -desireMapAngle+1, desireMapAngle-1);
      Serial.println(" Angle desirate : ==> roll : "+ String(roll_desired_angle) + "|| pitch : " + String(pitch_desired_angle) + "|| yaw : " + String(yaw_desired_angle));
    }

    void error(float actualRoll, float actualPitch, float actualYaw) {
      roll_error = roll_desired_angle - actualRoll;
      pitch_error = pitch_desired_angle - actualPitch;
      yaw_error = yaw_desired_angle - actualYaw;
    }

    void compute() {
      // Proportional term
      roll_pid_p = roll_kp * roll_error;
      pitch_pid_p = pitch_kp * pitch_error;
      yaw_pid_p = yaw_kp * yaw_error;

      // Integral term with anti-windup
      if (abs(roll_error) < 3) {
        roll_pid_i += roll_ki * roll_error * elapsedTime;
      }
      if (abs(pitch_error) < 3) {
        pitch_pid_i += pitch_ki * pitch_error * elapsedTime;
      }
      if (abs(yaw_error) < 3) {
        yaw_pid_i += yaw_ki * yaw_error * elapsedTime;
      }

      // Derivative term
      roll_pid_d = roll_kd * (roll_error - roll_previous_error) / elapsedTime;
      pitch_pid_d = pitch_kd * (pitch_error - pitch_previous_error) / elapsedTime;
      yaw_pid_d = yaw_kd * (yaw_error - yaw_previous_error) / elapsedTime;

      // Compute total PID
      roll_PID = roll_pid_p + roll_pid_i + roll_pid_d;
      pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d;
      yaw_PID = yaw_pid_p + yaw_pid_i + yaw_pid_d;

      // Constrain PID output
      roll_PID = constrain(roll_PID, -PIDContraint, PIDContraint);
      pitch_PID = constrain(pitch_PID, -PIDContraint, PIDContraint);
      yaw_PID = constrain(yaw_PID, -PIDContraint, PIDContraint);

      Serial.println(" roll pid :" +String(roll_PID) + " || pitch pid : " +String(pitch_PID) + "|| yaw pid:" + String(yaw_PID));
      rmbError();
    }

    void motorControl(int input_THROTTLE) {
      // Calculate PWM for each motor
      pwm_R_F =input_THROTTLE - roll_PID - pitch_PID + yaw_PID;
      pwm_R_B =input_THROTTLE - roll_PID + pitch_PID - yaw_PID;
      pwm_L_B =input_THROTTLE + roll_PID + pitch_PID + yaw_PID;
      pwm_L_F =input_THROTTLE + roll_PID - pitch_PID - yaw_PID;

      // Constrain motor PWM values
      pwm_R_F = constrain(pwm_R_F, minPWM, maxPWM);
      pwm_L_F = constrain(pwm_L_F, minPWM, maxPWM);
      pwm_R_B = constrain(pwm_R_B, minPWM, maxPWM);
      pwm_L_B = constrain(pwm_L_B, minPWM, maxPWM);
    }

    void rmbError() {
      roll_previous_error = roll_error;
      pitch_previous_error = pitch_error;
      yaw_previous_error = yaw_error;
    }

    void reset() {
      roll_previous_error = 0;
      pitch_previous_error = 0;
      yaw_previous_error = 0;
      roll_pid_i = 0;
      pitch_pid_i = 0;
      yaw_pid_i = 0;
    }
};