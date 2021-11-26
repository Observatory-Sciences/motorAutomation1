/*************************************************************************\
* Copyright (c) 2021 Aerotech, Inc.
* This file is distributed subject to a Software License Agreement found
* in the file LICENSE that is included with this distribution.
\*************************************************************************/

#ifndef Automation1MotorController_H
#define Automation1MotorController_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "Automation1MotorAxis.h"
#include "Include/Automation1.h"

#define MAX_AUTOMATION1_AXES                32
#define MAX_AUTOMATION1_TASK                32
#define PROFILE_MOVE_TASK_INDEX             2
#define PROFILE_MOVE_ABORT_TIMEOUT          1000
#define DATA_POINTS_PER_SECOND              1000
#define CONTROLLER_POLL_NUM_DATA_POINTS     10

// New params added
#define AUTOMATION1_C_AckAllString          "AUTOMATION1_C_ACKALL"	//ajc-osl
#define AUTOMATION1_C_VelocityString        "AUTOMATION1_C_VELOCITY"
#define AUTOMATION1_C_FErrorString          "AUTOMATION1_C_FERROR"
#define AUTOMATION1_C_ExecuteCommandString  "AUTOMATION1_C_EXECUTE_COMMAND"
#define AUTOMATION1_C_ProcUsageString       "AUTOMATION1_C_PROC_USAGE"
#define AUTOMATION1_C_EnabledTasksString    "AUTOMATION1_C_ENABLED_TASKS"
#define AUTOMATION1_C_TaskStateString       "AUTOMATION1_C_TASK_STATE"
#define NUM_AUTOMATION1_PARAMS 7


class epicsShareClass Automation1MotorController : public asynMotorController
{
public:
    // Member functions we override from the base class.
    Automation1MotorController(const char* portName, const char* hostName, int numAxes, double movingPollPeriod, double idlePollPeriod);
    ~Automation1MotorController();
    void report(FILE* fp, int level);
    Automation1MotorAxis* getAxis(asynUser* pasynUser);
    Automation1MotorAxis* getAxis(int axisNo);
    asynStatus poll();

    /* These are the methods that we override */
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);	//ajc-osl
    asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars, size_t *nActual);
    
    void createAsynParams(void);

    // These are functions for profile moves.
    asynStatus initializeProfile(size_t maxProfilePoints);
    asynStatus buildProfile();
    asynStatus executeProfile();
    asynStatus abortProfile();
    asynStatus readbackProfile();

protected:

    // Array of pointers to axis objects.
    Automation1MotorAxis** pAxes_;

    int AUTOMATION1_C_AckAll_;
    int AUTOMATION1_C_Velocity_;
    int AUTOMATION1_C_FError_;
    int AUTOMATION1_C_ExecuteCommand_;
    int AUTOMATION1_C_ProcUsage_;
    int AUTOMATION1_C_EnabledTasks_;
    int AUTOMATION1_C_TaskState_;
    int parameters[NUM_AUTOMATION1_PARAMS];

private:

    // An Automation1 Controller Handle used by the C API to
    // actually execute commands on the controller.
    Automation1Controller controller_;
    int availTaskCount_;
    int numPollDataPoints_ = CONTROLLER_POLL_NUM_DATA_POINTS;                       // Number of data points per data signal to collect during controller polling.
    int numPollDataSignals_ = 0;                                                    // This is updated in the constructor after each signal is successfully added.
    Automation1TaskStatus taskStatusArr[MAX_AUTOMATION1_TASK];

    // A handle that will be used to specify the data logged for
    // readbacks.
    Automation1DataCollectionConfig dataCollectionConfig_, pollDataConfig_;
    Automation1StatusConfig pollStatusConfig_;

    // Axes to be used in a profile move.
    std::vector<int> profileAxes_;

    // The resolution of the motor axes. Needed for profile motion 
    // because Automation1 works in EGU, not steps.
    std::vector<double> profileAxesResolutions_;

    // Automation1 error codes and messages must be acquired through
    // calls to the C API. To avoid duplicate code, we wrap calls 
    // to those functions and to asynPrint in this function.
    void logError(const char* driverMessage);

    friend class Automation1MotorAxis;
};

#endif
