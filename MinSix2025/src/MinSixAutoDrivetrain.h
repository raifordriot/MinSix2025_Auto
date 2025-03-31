/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       MinSixAutoDrivetrain.h                                    */
/*    Author:       Raiford G. Bonnell                                        */
/*    Created:      3/28/2025, 7:51:13 PM                                     */
/*    Description:  Drivetrain library for 6 wheel IQ drivetrain              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <math.h> 
#include "vex.h"

#ifndef MinSixAutoDrivetrain
#define MinSixAutoDrivetrain
class Min6AutoDrivetrain
{
    private:
        vex::brain m_Brain;
        vex::inertial m_BrainInertial;
        vex::motor m_RightDriveMotor;
        vex::motor m_LeftDriveMotor;
        
        int _inGearSize;
        int _outGearSize;
        double _wheelCircumference;
        double _maxMotorRPM;
        double _minMotorRPM;

        int GetQuad(double Heading);

    public:    
        enum LogLevels
        {
            None,
            CallsOnly,
            Verbose
        };
        LogLevels _logLevel;

        /// @brief 
        /// @param Brain 
        /// @param BrainInertial 
        /// @param RightDriveMotoer 
        /// @param LeftDriveMotor 
        Min6AutoDrivetrain(const vex::brain &Brain, 
                           const vex::inertial &BrainInertial,
                           const vex::motor &RightDriveMotoer, 
                           const vex::motor &LeftDriveMotor) 
                        : m_Brain(Brain)
                        , m_BrainInertial(BrainInertial)
                        , m_RightDriveMotor(RightDriveMotoer)
                        , m_LeftDriveMotor(LeftDriveMotor)
        {}

        /// @brief Set the output log level
        /// @param logLevel [LogLevels enum] log level value
        void Set_LogLevel(LogLevels logLevel)
        {
            _logLevel = logLevel;
        }

        /// @brief Smallest motor velocity in RPMs which will result in bot motion
        /// @param minimalVelocity [double] - minimal velocity value
        void Set_MINIMAL_MOTOR_RPM(double minMotorRPM)
        {
            _minMotorRPM = minMotorRPM;
        }

        /// @brief Size in tooth count of the input gear
        /// @param inGearSize [double] gear tooth count
        void Set_IN_GEAR_SIZE(int inGearSize)
        {
            _inGearSize = inGearSize;
        }

        /// @brief Size in tooth count of the output gear
        /// @param inGearSize [double] gear tooth count
        void Set_OUT_GEAR_SIZE(int outGearSize)
        {
            _outGearSize = outGearSize;
        }

        /// @brief Circumference of drive wheel/s in mm
        /// @param wheelCircumference [double] circunference of drive wheel
        void Set_WHEEL_CIRCUMFERENCE(double wheelCircumference)
        {
            _wheelCircumference = wheelCircumference;
        }

        /// @brief Maximum motor RPM value to be used in this library
        /// @param maxMotorRPM [int] maximun value
        void Set_MAX_MOTOR_RPM(double maxMotorRPM)
        {
            _maxMotorRPM = maxMotorRPM;
        }

        void CalibrateGyro();
        void TurnToHeading(double Heading, double TurnVelocity, double TimeOut, double HeadingTolerance, int AccertionSteps = 10);
};
#endif