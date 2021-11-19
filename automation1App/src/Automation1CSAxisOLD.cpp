
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include <epicsExport.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "Automation1MotorAxis.h"
#include "Automation1MotorController.h"
#include "Automation1CSAxis.h"
#include "Include/Automation1.h"

/** Creates a new Automation1 CS axis object.
  * \param[in] pC     Pointer to the Automation1MotorController to which this axis belongs.
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  */
Automation1CSAxis::Automation1CSAxis(Automation1MotorController* pC, int axisNo, int globalAxisIndex, int totalNumAxes, int globalStartIndex)
    : Automation1MotorAxis(pC, axisNo),
    pC_(pC)
{
    Automation1_StatusConfig_Create(&(statusConfig_));
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_AxisStatus, 0);
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_DriveStatus, 0);
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_ProgramPositionFeedback, 0);
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_ProgramVelocityFeedback, 0);
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_AxisFault, 0);
    Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisNo, Automation1AxisStatusItem_PositionError, 0);

    // Gain Support is required for setClosedLoop to be called
    setIntegerParam(pC->motorStatusGainSupport_, 1);
    
    // Set the index values for the globals to use.
    globalRealReadIndex_  = globalAxisIndex;
    globalRealWriteIndex_ = globalAxisIndex + 1;
    
    

    if(!Automation1CSAxis::setupRun_)
    {

    	//This block should only run for the first instance of the class.
	Automation1CSAxis::setupRun_ = true;
	static const char *functionName = "Automation1CSAxis::Automation1CSAxis()";
    	
    	
	// Write to $iglobal[0,1] for NumAxes and globalStartIndex.
	std::string strTmp1 = "$iglobal[0]="+ std::to_string(totalNumAxes);
	std::string strTmp2 = "$iglobal[1]="+ std::to_string(globalStartIndex);
	const char* aeroScriptCmd1 = strTmp1.c_str();
	const char* aeroScriptCmd2 = strTmp2.c_str();

	if (!Automation1_Command_Execute(pC->controller_, 1, aeroScriptCmd1))
	{
		printf("Automation1:%s: Error Could not set value of numAxes to a controller $iglobal variable.\n",
		       functionName);
	}
	if (!Automation1_Command_Execute(pC->controller_, 1, aeroScriptCmd2))
	{
		printf("Automation1:%s: Error Could not set value of globalStartIndex to a controller $iglobal variable.\n",
		       functionName);
	}

	// Call forward monitor script after stopping anything running on that task.
	Automation1_Task_ProgramStop(pC_->controller_, FORWARD_MONITOR_TASK_INDEX, 0);
	const char* forward_monitor_script = "forward_transform_monitor.ascript\0";
	if( !Automation1_Task_ProgramRun(pC->controller_, FORWARD_MONITOR_TASK_INDEX, forward_monitor_script))
	{
		printf("Automation1:%s: Error Could not start the forward transformation monitoring script on the controller on task %d. (Is the controller configured to have %d tasks enabled?)\n",
		       functionName,FORWARD_MONITOR_TASK_INDEX,FORWARD_MONITOR_TASK_INDEX);
	}
	
    }
    
    
}

// Destructor.
Automation1CSAxis::~Automation1CSAxis()
{
    Automation1_StatusConfig_Destroy(statusConfig_);
    //Stop the forward monitoring script.
    Automation1_Task_ProgramStop(pC_->controller_, FORWARD_MONITOR_TASK_INDEX, 0);
}

extern "C" int Automation1CreateCSAxis(const char* portName, int numAxes, int offset, int globalStartIndex)
{
    Automation1MotorController *pC;
    static const char *functionName = "Automation1CreateCSAxis";
    pC = (Automation1MotorController*) findAsynPortDriver(portName);
    if (!pC)
    {
        printf("Automation1:%s: Error port %s not found\n",
               functionName, portName);
        return asynError;
    }
    int maxIndex = globalStartIndex + 2*numAxes;	//Check if there are enough globals for all axes starting from the chosen index. Throw an error if there isn't.
    if ( maxIndex > 255)
    {
    	printf("Automation1:%s: Error global indices run to 255 and the chosen configuration has a maximum index of %d. Reduce numAxes or globalStartIndex to avoid this error.\n",
               functionName, maxIndex);
        return asynError;
    }
	
    
    int axisIndex = globalStartIndex;
    for (int axis = 0; axis < numAxes; axis++)
    {
    	new Automation1CSAxis(pC, (axis+offset), axisIndex, numAxes, globalStartIndex);
    	axisIndex += 2;	//1 for read/RBV, 1 for write/DMD. In that order.
    }
    
    
    return(asynSuccess);
}

void Automation1CSAxis::logError(const char* driverMessage)
{
    int errorCode = Automation1_GetLastError();
    char errorMessage[1024];
    Automation1_GetLastErrorMessage(errorMessage, 1024);

    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "Driver: Automation1. Function Message: %s. Axis: %d. API Error Code: %d. API Error Message: %s\n",
              driverMessage,
              axisNo_,
              errorCode,
              errorMessage);

    return;
}


/** Move the motor by a relative amount or to an absolute position.
  * \param[in] position     The absolute position to move to (if relative=0) or the relative distance to move
  *                          by (if relative=1). Units=steps.
  * \param[in] relative     Flag indicating relative move (1) or absolute move (0).
  * \param[in] minVelocity  The initial velocity. Not used by Automation1.
  * \param[in] maxVelocity  The target velocity for the move.  Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec.
  */
asynStatus Automation1CSAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
    double resolution;
    pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &resolution);
    double adjustedAcceleration = acceleration * resolution;
    double adjustedVelocity = maxVelocity * resolution;
    double adjustedPosition = position * resolution;
    bool moveSuccessful;
    static const char *functionName = "Automation1CSAxis::move";
    
	
    if (!Automation1_Command_SetupAxisRampValue(pC_->controller_,
                                                1,
                                                axisNo_,
                                                Automation1RampMode_Rate,
                                                adjustedAcceleration,
                                                Automation1RampMode_Rate,
                                                adjustedAcceleration))
    {
        logError("Failed to set acceleration prior to move.");
        return asynError;
    }
    
    // write position demand value for this axis to $rglobal[j] and to then call the Aeroscript to manage transforms and movement.
	std::string strWritePos = "$rglobal["+std::to_string(globalRealWriteIndex_)+"]="+ std::to_string(adjustedPosition);
	const char* cchrWritePos = strWritePos.c_str();
	const char* aeroScriptName = "SlitMove.ascript\0";
	
	if (!Automation1_Command_Execute(pC_->controller_, 1, cchrWritePos))
	{
		printf("Automation1:%s: Error Could not set commanded position value to a controller $rglobal variable.\n",
		       functionName);
	}
	
	moveSuccessful = Automation1_Task_ProgramRun(pC_->controller_, 1, aeroScriptName);


    if (moveSuccessful)
    {
        return asynSuccess;
    }
    else
    {
        logError("Failed to move axis.");
        return asynError;
    }
}



/** Move the motor at a fixed velocity until told to stop.
  * \param[in] minVelocity  The initial velocity. Not used by Automation1.
  * \param[in] maxVelocity  The target velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec.
  */
asynStatus Automation1CSAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
    double resolution;
    pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &resolution);
    double adjustedAcceleration = acceleration * resolution;
    double adjustedVelocity = maxVelocity * resolution;
    

    if (!Automation1_Command_SetupAxisRampValue(pC_->controller_,
                                                1,
                                                axisNo_,
                                                Automation1RampMode_Rate,
                                                adjustedAcceleration,
                                                Automation1RampMode_Rate,
                                                adjustedAcceleration))
    {
        logError("Failed to set acceleration prior to moveVelocity (jog).");
        return asynError;
    }

    if (Automation1_Command_MoveFreerun(pC_->controller_, 1, &axisNo_, 1, &adjustedVelocity, 1))
    {
        return asynSuccess;
    }
    else
    {
        logError("moveVelocity (jog) failed.");
        return asynError;
    }
}


/** Polls the axis.
  *
  * This function reads the motor position, encoder position, and the following statuses: direction,
  * done, high and low limit, at home, following error, moving, and comms error.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false).
*/
asynStatus Automation1CSAxis::poll(bool* moving)
{
    bool pollSuccessfull = true;
    double results[5];
    int axisStatus;
    int driveStatus;
    int enabled;
    double programPositionFeedback;
    double programVelocityFeedback;
    double programPositionCommand;
    double countsPerUnitParam;
    int axisFaults;
    int done;
    double resolution;
    pC_->getDoubleParam(axisNo_, pC_->motorRecResolution_, &resolution);
    static const char *functionName = "Automation1CSAxis::poll";
    double valPos;
    std::string strReadPos = "$rreturn[0]=$rglobal["+ std::to_string(globalRealReadIndex_) +"]";
    const char* cchrReadPos = strReadPos.c_str();


    // This actually retrieves the status items from the controller.
    if (!Automation1_Status_GetResults(pC_->controller_,
                                       statusConfig_,
                                       results,
                                       (int)(sizeof(results) / sizeof(double))))
    {
        pollSuccessfull = false;
        goto skip;
    }
    
    // read from $rglobal[i] and set position from it.
    
    
    if( !Automation1_Command_ExecuteAndReturnAeroScriptReal(pC_->controller_, 1, cchrReadPos, &valPos) )
    {
    	printf("Automation1:%s: Error Could not read axis position from global variable.\n",
		       functionName);
    }

	       
    
    axisStatus = results[0];
    driveStatus = results[1];
    //programPositionFeedback = results[2];
    programPositionFeedback = valPos;
    programVelocityFeedback = results[3];
    axisFaults = (int)results[4];

    if (!Automation1_Parameter_GetAxisValue(pC_->controller_,
        axisNo_,
        Automation1AxisParameterId_CountsPerUnit,
        &countsPerUnitParam))
    {
        pollSuccessfull = false;
        goto skip;
    }

    enabled = driveStatus & Automation1DriveStatus_Enabled;
    setIntegerParam(pC_->motorStatusPowerOn_, enabled);
    setDoubleParam(pC_->motorPosition_, programPositionFeedback / resolution);
    setDoubleParam(pC_->motorEncoderPosition_, programPositionFeedback * countsPerUnitParam);

    done = axisStatus & Automation1AxisStatus_MotionDone;
    setIntegerParam(pC_->motorStatusDone_, done);
    if (done)
    {
        *moving = false;
        setIntegerParam(pC_->motorStatusMoving_, 0);
    }
    else
    {
        if (programVelocityFeedback != 0)
        {
            *moving = true;
            setIntegerParam(pC_->motorStatusMoving_, 1);
            if (programVelocityFeedback > 0)
            {
                setIntegerParam(pC_->motorStatusDirection_, 1);
            }
            else
            {
                setIntegerParam(pC_->motorStatusDirection_, 0);
            }
        }
        else
        {
            *moving = false;
            setIntegerParam(pC_->motorStatusMoving_, 0);
        }
    }

    if ((axisFaults & Automation1AxisFault_CwEndOfTravelLimitFault) ||
        (axisFaults & Automation1AxisFault_CwSoftwareLimitFault))
    {
        setIntegerParam(pC_->motorStatusHighLimit_, 1);
    }
    else
    {
        setIntegerParam(pC_->motorStatusHighLimit_, 0);
    }

    if ((axisFaults & Automation1AxisFault_CcwEndOfTravelLimitFault) ||
        (axisFaults & Automation1AxisFault_CcwSoftwareLimitFault))
    {
        setIntegerParam(pC_->motorStatusLowLimit_, 1);
    }
    else
    {
        setIntegerParam(pC_->motorStatusLowLimit_, 0);
    }

    if (axisFaults & Automation1AxisFault_PositionErrorFault)
    {
        setIntegerParam(pC_->motorStatusFollowingError_, 1);
    }
    else
    {
        setIntegerParam(pC_->motorStatusFollowingError_, 0);
    }

    setIntegerParam(pC_->motorStatusHomed_, axisStatus & Automation1AxisStatus_Homed);		// pC_->motorStatusAtHome_ CHANGED TO pC_->motorStatusHomed_ - ajc-osl

skip:

    setIntegerParam(pC_->motorStatusCommsError_, pollSuccessfull ? 0 : 1);
    callParamCallbacks();
    if (!pollSuccessfull)
    {
        logError("Poll failed.");
    }

    return pollSuccessfull ? asynSuccess : asynError;
}

// Code for iocsh registration
static const iocshArg Automation1CreateCSAxisArg0 = { "Port name", iocshArgString };
static const iocshArg Automation1CreateCSAxisArg1 = { "Number of CS axes to create", iocshArgInt };
static const iocshArg Automation1CreateCSAxisArg2 = { "Number of regular axes created", iocshArgInt };
static const iocshArg Automation1CreateCSAxisArg3 = { "Start index for controller global variables", iocshArgInt };
static const iocshArg* const Automation1CreateCSAxisArgs[] = { &Automation1CreateCSAxisArg0,
                                                                   &Automation1CreateCSAxisArg1,
                                                                   &Automation1CreateCSAxisArg2,
                                                                   &Automation1CreateCSAxisArg3 };
static const iocshFuncDef Automation1CreateCSAxisDef = { "Automation1CreateCSAxis", 4, Automation1CreateCSAxisArgs };
static void Automation1CreateCSAxisCallFunc(const iocshArgBuf* args)
{
    Automation1CreateCSAxis(args[0].sval, args[1].ival, args[2].ival, args[3].ival);
}

static void Automation1CSRegister(void)
{
    iocshRegister(&Automation1CreateCSAxisDef, Automation1CreateCSAxisCallFunc);
}

extern "C" {
    epicsExportRegistrar(Automation1CSRegister);
}


