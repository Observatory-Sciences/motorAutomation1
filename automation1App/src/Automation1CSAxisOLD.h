/*************************************************************************\
* Copyright (c) 2021 Aerotech, Inc.
* This file is distributed subject to a Software License Agreement found
* in the file LICENSE that is included with this distribution.
\*************************************************************************/

#ifndef Automation1CSAxis_H
#define Automation1CSAxis_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "Automation1MotorAxis.h"
#include "Include/Automation1.h"

#define FORWARD_MONITOR_TASK_INDEX 3

class Automation1MotorController;
class Automation1MotorAxis;

class epicsShareClass Automation1CSAxis : public Automation1MotorAxis
{
public:
    // Member functions we override from the base class.
    Automation1CSAxis(Automation1MotorController* pC, int axis, int globalAxisIndex, int totalNumAxes, int globalStartIndex);
    ~Automation1CSAxis();

    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
    asynStatus poll(bool* moving);



private:
    // Pointer to asynMotorController to which the axis belongs.
    Automation1MotorController* pC_;
    
    static bool setupRun_;
    
    // Global variable index in Automation1 controller to use. Uses that index and index+1
    int globalRealReadIndex_, globalRealWriteIndex_;

    // The config is used by certain functions in the Automation1 C API to 
    // get status items from the controller.  This is needed for polling.
    Automation1StatusConfig statusConfig_;

    // Automation1 error codes and messages must be acquired through
    // calls to the C API.  To avoid duplicate code, we wrap calls 
    // to those functions and to asynPrint in this function.
    void logError(const char* driverMessage);

    friend class Automation1MotorController;
};

bool Automation1CSAxis::setupRun_ = false;

#endif
