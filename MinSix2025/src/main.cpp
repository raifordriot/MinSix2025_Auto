/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       MinSix2025_Auto.cpp                                                  */
/*    Author:       Raiford G. Bonnell                                                    */
/*    Created:      3/28/2025, 7:51:13 PM                                     */
/*    Description:  IQ2 project                                               */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "MinSixAutoDrivetrain.h"

using namespace vex;

//Brain and PORT configuration
vex::brain gBrain;
vex::inertial gBrainInertial;
vex::motor gRightDriveMotor(PORT12, false);
vex::motor gLeftDriveMotor(PORT6, true);
Min6AutoDrivetrain gDrivetrain(gBrain, gBrainInertial, gRightDriveMotor, gLeftDriveMotor);
vex::touchled gSelectStopLed(PORT10);
vex::touchled gStartLed(PORT11);
vex::optical gFrontOptical(PORT1);
vex::distance gDistance(PORT7);

//Globals
enum StatesOfBot 
{
    LOW_BATTERY,
    READY,
    RUNNING
};
StatesOfBot gBotState = StatesOfBot::LOW_BATTERY;

enum RoutinsToRun
{
    Routine_None,
    Routine_1,
    Routine_2
};
RoutinsToRun gRoutin = RoutinsToRun::Routine_None;

//Define Functions
void SelectStop_Pressed();
void Start_Pressed();
void Routine_One();
void Routine_Two();

int main() 
{
    //Setup touch leds eventhandlers
    gSelectStopLed.pressed(SelectStop_Pressed);
    gStartLed.pressed(Start_Pressed);

    //Check Battery Level
    if (gBrain.Battery.capacity() >= 80)
        gBotState = StatesOfBot::READY;
    
    //If battery is low play siren until overridden
    if (gBotState == StatesOfBot::LOW_BATTERY)
    {
        gSelectStopLed.setColor(vex::colorType::red);
        printf("Battery Level at %d%", gBrain.Battery.capacity());
        while(true)
        {
            gBrain.playSound(vex::soundType::siren);
            wait(1.0f, vex::timeUnits::sec);
            if (gBotState == StatesOfBot::READY)
                break;
        }
        printf("Low battery override!");
    }
    gSelectStopLed.setColor(vex::colorType::white);
    gStartLed.setColor(vex::colorType::none);

    //Configure drivetrain
    gDrivetrain.Set_LogLevel(Min6AutoDrivetrain::LogLevels::Verbose);
    gDrivetrain.Set_MAX_MOTOR_RPM(110.0f);
    gDrivetrain.Set_MINIMAL_MOTOR_RPM(5.0f);
    gDrivetrain.Set_WHEEL_CIRCUMFERENCE(230.0f);
    gDrivetrain.Set_IN_GEAR_SIZE(48);
    gDrivetrain.Set_OUT_GEAR_SIZE(24);
    gDrivetrain.CalibrateGyro();
   
    while(1) 
    {    
        // Allow other tasks to run
        this_thread::sleep_for(100);
    }
}

#pragma region Handlers
void SelectStop_Pressed()
{
    printf("Select/Stop Pressed\n");
    if (gBotState == StatesOfBot::READY)
    { 
        switch (gRoutin)
        {
            case RoutinsToRun::Routine_None:
                gSelectStopLed.setColor(vex::colorType::blue);
                gStartLed.setColor(vex::colorType::green);
                gRoutin = RoutinsToRun::Routine_1;
                break;
            case RoutinsToRun::Routine_1:
                gSelectStopLed.setColor(vex::colorType::orange);
                gStartLed.setColor(vex::colorType::green);
                gRoutin = RoutinsToRun::Routine_2;
                break;
            case RoutinsToRun::Routine_2:
                gRoutin = RoutinsToRun::Routine_None;
                gStartLed.setColor(vex::colorType::none);
                gSelectStopLed.setColor(vex::colorType::white);
                break;
        }
    }
    else if (gBotState == StatesOfBot::RUNNING)
    {
        gRoutin = RoutinsToRun::Routine_None;
        gSelectStopLed.setColor(vex::colorType::white);
        gBotState = StatesOfBot::READY;
    }
    else 
    {
        gBotState = StatesOfBot::READY;
    }
}

void Start_Pressed()
{
    printf("Start Pressed\n");
    if (gBotState == StatesOfBot::READY)
    {
        switch (gRoutin)
        {
            case RoutinsToRun::Routine_1:
                gBotState = StatesOfBot::RUNNING;
                gStartLed.setColor(vex::colorType::none);
                gSelectStopLed.setColor(vex::colorType::red);
                Routine_One();
                gStartLed.setColor(vex::colorType::green);
                gSelectStopLed.setColor(vex::colorType::blue);
                gBotState = StatesOfBot::READY;
                break;
            case RoutinsToRun::Routine_2:
                gBotState = StatesOfBot::RUNNING;
                gStartLed.setColor(vex::colorType::none);
                gSelectStopLed.setColor(vex::colorType::red);
                Routine_Two();
                gStartLed.setColor(vex::colorType::green);
                gSelectStopLed.setColor(vex::colorType::orange);
                gBotState = StatesOfBot::READY;
                break;
            case RoutinsToRun::Routine_None:
                gStartLed.setColor(vex::colorType::none);
                gSelectStopLed.setColor(vex::colorType::white);
                break;
        }
    }
}
#pragma endregion

#pragma region Routins
void Routine_One()
{
    gBrain.Screen.setCursor(3, 1);
    gBrain.Screen.print("Running 1");
    gDrivetrain.TurnToHeading(90.0f, 50.0f, 5.0f, 0.1f);
    gBrain.Screen.setCursor(3, 10);
    gBrain.Screen.clearLine();

    return;
}

void Routine_Two()
{
    gBrain.Screen.setCursor(3, 1);
    gBrain.Screen.print("Running 2");
    wait(2.0f, vex::timeUnits::sec);
    gBrain.Screen.setCursor(3, 10);
    gBrain.Screen.clearLine();

    return;
}
#pragma endregion