#ifndef PID_CONTROLLER
#define PID_CONTROLLER
class PIDController 
{
private:
    double Kp, Ki, Kd;
    double previousError, integral;

public:
    /// @brief Class to calculate error signal using PID method
    /// @param p [double] Proportional Coefficient
    /// @param i [double] Integral Coefficient
    /// @param d [double] Derivative Coefficient
    PIDController(double p, double i, double d) : Kp(p), Ki(i), Kd(d), previousError(0), integral(0) {}

    /// @brief Calculate a new control signal 
    /// @param error [double] Measured error
    /// @return [double] Calculated control signal
    double calculateControlSignal(double error) 
    {
        //calculate the proportional value by applying the proportional coefficient to the error. 
        double proportional = Kp * error;
        
        //add the current error to the integral value.
        integral += error;
        
        //calculate the derivative value by comparing the current error to the past error
        double derivative = error - previousError;

        //calculate the output by adding the proportional value to the integral (with coefficient) and adding the derivative (with coefficient) to the sum.
        double output = proportional + Ki * integral + Kd * derivative;

        //record the error to be used as the past error in the next call
        previousError = error;

        return output;
    }
};
#endif
