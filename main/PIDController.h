     #ifndef PIDCONTROLLER_H
     #define PIDCONTROLLER_H

     class PIDController {
     private:
         double Kp, Ki, Kd;
         double setpoint;
         double integral;
         double previous_error;

     public:
         PIDController(double kp, double ki, double kd, double sp);
         double compute(double current_value);
         void setCoefficients(double kp, double ki, double kd);
         void setSetpoint(double sp);
         double getSetpoint();
     };

     #endif // PIDCONTROLLER_H
