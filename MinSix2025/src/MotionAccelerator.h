#include "vex.h"

#ifndef Motion_Acc
#define Motion_Acc
class MotionAccelerator
{
    private:
    int _step = 0;
    int _accStepCount;
    int _steadyStepCount;
    int _decelerationStepCount;
    double _maxMotorRPM;
    double _minMotorRPM;

    public:
    /// @brief Constructor for Accelerator class
    /// @param AccStepCount[int] Interger value indicating the total number of acceleration steps
    /// @param SteadyStepCount[int] Integer value indicating the total number of max. velocity steps
    /// @param DecelerationStepCount[int] integer value indicating the number of deceleration steps.
    /// @param MaxMotorRPM[double] Max. Motor RPM to be accelerated to. 
    /// @param MinMotorRPM[double] Min. Motor RPM to be deceleration to.
    MotionAccelerator(int AccStepCount, int SteadyStepCount, int DecelerationStepCount, double MaxMotorRPM, double MinMotorRPM) : 
    _accStepCount(AccStepCount), 
    _steadyStepCount(SteadyStepCount),
    _decelerationStepCount(DecelerationStepCount),
    _maxMotorRPM(MaxMotorRPM),
    _minMotorRPM(MinMotorRPM)
    {}

    /// @brief Get the next acceration value
    /// @return [double] Next acceleration value.
    double GetNextStepRPM()
    {
        double NextStepRPM;

        _step++;
        if ((_step > _accStepCount) && (_step <= (_accStepCount + _steadyStepCount))) 
        {
            return _maxMotorRPM;
        }
        if ((_step > (_accStepCount + _steadyStepCount)) && (_step <= (_accStepCount + _steadyStepCount + _decelerationStepCount)))
        { //Deceleration
            NextStepRPM = _maxMotorRPM - (((double)(_step - (_accStepCount + _steadyStepCount)) / (double)_decelerationStepCount) * _maxMotorRPM);
        }
        else
        { //Acceleration
            NextStepRPM = ((double)_step / (double)_accStepCount) * _maxMotorRPM;
        }
        if (NextStepRPM < _minMotorRPM) NextStepRPM = _minMotorRPM;

        return NextStepRPM;
    }

    int GET_StepCount()
    {
        return _step;
    }
};
#endif
