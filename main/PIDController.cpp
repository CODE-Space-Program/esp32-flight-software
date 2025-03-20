     #include "PIDController.h"

     PIDController::PIDController(double kp, double ki, double kd, double sp)
         : Kp(kp), Ki(ki), Kd(kd), setpoint(sp), integral(0), previous_error(0) {}

     double PIDController::compute(double current_value) {
         double error = setpoint - current_value;
         integral += error;
         double derivative = error - previous_error;
         previous_error = error;
         return Kp * error + Ki * integral + Kd * derivative;
     }

     void PIDController::setCoefficients(double kp, double ki, double kd) {
         Kp = kp;
         Ki = ki;
         Kd = kd;
     }

     void PIDController::setSetpoint(double sp) {
         setpoint = sp;
     }

     double PIDController::getSetpoint() {
         return setpoint;
     }
