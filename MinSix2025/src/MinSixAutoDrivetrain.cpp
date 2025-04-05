/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       MinSixAutoDrivetrain.cpp                                    */
/*    Author:       Raiford G. Bonnell                                        */
/*    Created:      3/28/2025, 7:51:13 PM                                     */
/*    Description:  Drivetrain library for 6 wheel IQ drivetrain              */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "MinSixAutoDrivetrain.h"
#include "PIDController.h"
#include "MotionAccelerator.h"

void Min6AutoDrivetrain::CalibrateGyro()
{
    if (_logLevel > LogLevels::None) printf("[CalibrateGyro]\n");
    vex::wait(250, vex::timeUnits::msec);
    m_Brain.Timer.reset();
    m_Brain.Screen.setCursor(1, 1);
    m_Brain.Screen.print("Calibrating...");
    if (_logLevel == LogLevels::Verbose) printf("Start Calibration\n");
    
    m_BrainInertial.calibrate();
    while (m_BrainInertial.isCalibrating()) 
    {
        vex::wait(250, vex::timeUnits::msec);
    }
    m_Brain.Screen.clearScreen();
    m_Brain.Screen.setCursor(1, 1);
    m_Brain.Screen.print("Ready");
    if (_logLevel == LogLevels::Verbose) printf("End Calibration after %f seconds\n", m_Brain.Timer.value());

    return;
}

/// @brief Turn Bot to desired heading if the bots current heading is not within the heading toleracne.
/// @param Heading [Degrees]The desired heading for the bot.
/// @param TurnVelocity [RPM]The top motor RPM to be used during the turn.
/// @param TimeOut [ms]Time allotted to complete the turn.
/// @param HeadingTolerance [deg]Allowed error in turn.
/// @param AccertionSteps [int][Optional default value is 10]Number of steps to be used to bring motors up to speed.
void Min6AutoDrivetrain::TurnToHeading(double Heading, double TurnVelocity, double TimeOut, double HeadingTolerance, int AccertionSteps)
{
    if (_logLevel > LogLevels::None)
    {
        printf("-----[TurnToHeading]-----\n");
        printf("Heading: %f\n", Heading);
        printf("TurnVelocity: %f\n", TurnVelocity);
        printf("TimeOut: %f\n", TimeOut);
        printf("HeadingTolerance: %f\n", HeadingTolerance);
        printf("-------------------------\n");
    }

    //Is bot current heading within tolerance?
    bool IsTurnRequired = false;
    int CurrentQuad = GetQuad(m_BrainInertial.heading(vex::rotationUnits::deg));
    int DesiredQuad = GetQuad(Heading);
    if (_logLevel == LogLevels::Verbose)
    {
        printf("CurrentQuad: %d\n", CurrentQuad);
        printf("DesiredQuad: %d\n", DesiredQuad);
    }
    if ((CurrentQuad == 4) && (DesiredQuad == 1))
    {
        IsTurnRequired = (((360 - m_BrainInertial.heading(vex::rotationUnits::deg) + Heading)) > HeadingTolerance);
    }
    else
    {    
        if ((CurrentQuad == 1) && (DesiredQuad == 4))
        {
            IsTurnRequired = (((360 - Heading) + m_BrainInertial.heading(vex::rotationUnits::deg)) > HeadingTolerance);
        }
        else
        {
            IsTurnRequired = ((abs(Heading - m_BrainInertial.heading(vex::rotationUnits::deg))) > HeadingTolerance);    
        }
    }
    if (IsTurnRequired)
    {
        //Setup PID signal Calculater
        PIDController pid(1.0f, 0.0f, 0.0f);

        //Setup Acceration control
        MotionAccelerator ma(AccertionSteps, 1000, 0, TurnVelocity, _minMotorRPM);

        //Calculate Turn amount and direction
        double d1 = abs(Heading - m_BrainInertial.heading(vex::rotationUnits::deg));
        double d2 = 360 - d1;
        double CWTurnAmount = 0.0f;
        double CCWTurnAmount = 0.0f;
        if (m_BrainInertial.heading(vex::rotationUnits::deg) < Heading)
        {
            CWTurnAmount = d1;
            CCWTurnAmount = d2;
        }
        else
        {
            CWTurnAmount = d2;
            CCWTurnAmount = d1;
        }
        if (_logLevel == LogLevels::Verbose)
        {
            printf("CWTurnAmount: %f\n", CWTurnAmount);
            printf("CCWTurnAmount: %f\n", CCWTurnAmount);
        }
        //Zero out bot rotation
        m_BrainInertial.setRotation(0, vex::rotationUnits::rev);
        if (CWTurnAmount < CCWTurnAmount)
        {
            m_Brain.resetTimer();
            if (_logLevel > LogLevels::None) printf("Turn Clockwise\n");
            if (_logLevel == LogLevels::Verbose) printf("Requested, Right Actual, Left Actual, Heading\n");
            m_RightDriveMotor.spin(vex::directionType::rev);
            m_LeftDriveMotor.spin(vex::directionType::fwd);
            while (m_BrainInertial.rotation(vex::rotationUnits::deg) < CWTurnAmount)
            {
                double MotorVelocity;
                if (ma.GET_StepCount() < AccertionSteps)
                    MotorVelocity = ma.GetNextStepRPM();
                else
                    MotorVelocity = ((pid.calculateControlSignal(abs(CWTurnAmount - m_BrainInertial.rotation(vex::rotationUnits::deg)))) / CWTurnAmount) * TurnVelocity;
                if (MotorVelocity < _minMotorRPM) MotorVelocity = _minMotorRPM;
                m_RightDriveMotor.setVelocity((MotorVelocity * -1), vex::velocityUnits::rpm);
                m_LeftDriveMotor.setVelocity(MotorVelocity, vex::velocityUnits::rpm);   
                if (_logLevel == LogLevels::Verbose)
                    printf("%f, %f, %f, %f\n", 
                        MotorVelocity, 
                        abs(m_RightDriveMotor.velocity(vex::velocityUnits::rpm)), 
                        abs(m_LeftDriveMotor.velocity(vex::velocityUnits::rpm)),
                        m_BrainInertial.heading(vex::rotationUnits::deg));  
                if (m_Brain.Timer.time(vex::timeUnits::sec) > TimeOut)
                {
                    if (_logLevel > LogLevels::None) 
                        printf("**TIMED OUT**\n");
                    break;
                }
                wait(20, vex::timeUnits::msec);
            }
            m_RightDriveMotor.stop();
            m_LeftDriveMotor.stop();
        }
        else
        {
            m_Brain.resetTimer();
            if (_logLevel > LogLevels::None) printf("Turn Counter Clockwise\n");
            if (_logLevel == LogLevels::Verbose) printf("Requested, Right Actual, Left Actual, Heading\n");
            m_RightDriveMotor.spin(vex::directionType::fwd);
            m_LeftDriveMotor.spin(vex::directionType::rev);
            while (abs(m_BrainInertial.rotation(vex::rotationUnits::deg)) < CCWTurnAmount)
            {
                double MotorVelocity;
                if (ma.GET_StepCount() < AccertionSteps)
                    MotorVelocity = ma.GetNextStepRPM();
                else
                    MotorVelocity = ((pid.calculateControlSignal(abs(CCWTurnAmount - abs(m_BrainInertial.rotation(vex::rotationUnits::deg))))) / CCWTurnAmount) * TurnVelocity;
                if (MotorVelocity < _minMotorRPM) MotorVelocity = _minMotorRPM;
                m_RightDriveMotor.setVelocity(MotorVelocity, vex::velocityUnits::rpm);
                m_LeftDriveMotor.setVelocity((MotorVelocity * -1), vex::velocityUnits::rpm);   
                if (_logLevel == LogLevels::Verbose)
                    printf("%f, %f, %f, %f, %f\n", 
                        MotorVelocity, 
                        abs(m_RightDriveMotor.velocity(vex::velocityUnits::rpm)), 
                        abs(m_LeftDriveMotor.velocity(vex::velocityUnits::rpm)),
                        m_BrainInertial.heading(vex::rotationUnits::deg),  
                        m_BrainInertial.rotation(vex::rotationUnits::deg));
                if (m_Brain.Timer.time(vex::timeUnits::sec) > TimeOut)
                {
                    if (_logLevel > LogLevels::None) 
                        printf("**TIMED OUT**\n");
                    break;
                }
                wait(20, vex::timeUnits::msec);
            }
            m_RightDriveMotor.stop();
            m_LeftDriveMotor.stop();
        }
        if (_logLevel > LogLevels::None) 
        {
            printf("Final Heading: %f degress\n", m_BrainInertial.heading(vex::rotationUnits::deg));
            printf("Time To Complete: %fs\n", m_Brain.Timer.time(vex::timeUnits::sec));
        }
    }
    else
    {
        if (_logLevel > LogLevels::None) printf("No turn needed\n");
    }

    return;
}

/// @brief Utility function to calculate which quadrent the given heading is in.
/// @param Heading [Dgrees]Heading to evaluate.
/// @return integer repersenting the quadrent.
int Min6AutoDrivetrain::GetQuad(double Heading)
{
    if ((Heading >= 0) && (Heading <= 90))
        return 1;
    if ((Heading > 90) && (Heading <= 180))
        return 2;
    if ((Heading > 180) && (Heading <= 270))
        return 3;
    if ((Heading > 270) && (Heading < 360))
        return 4;

    return 0;
}