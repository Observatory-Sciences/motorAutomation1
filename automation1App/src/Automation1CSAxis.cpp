#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <iocsh.h>
#include <epicsThread.h>

#include <asynOctetSyncIO.h>

#include <epicsExport.h>

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "Automation1MotorAxis.h"
#include "Automation1MotorController.h"
#include "Include/Automation1.h"
#include "Automation1CSAxis.h"

#include <iostream>
#include <vector>
#include <algorithm>

std::vector<Automation1CSAxis*> Automation1CSAxis::csAxisList_;

//#include <chrono>
//#include <thread>
//std::this_thread::sleep_for(std::chrono::milliseconds(x));


/** Creates a new Automation1 CS axis object.
  * \param[in] pC     			    Pointer to the Automation1MotorController to which this axis belongs.
  * \param[in] transformScriptName	The name of the Aeroscript file on the controller which will handle the FWD/INV kinetic transformations.
  * \param[in] CS_Type		        The type of CS this axis is part of.
  * \param[in] CS_Name			    The Co-ordinate system to which this virtual axis belongs. Virtual axes are grouped by this parameter.
  * \param[in] globalReadIndexVel   The $rglobal variable index on the controller used for the virtual motor's velocity readback value (RBV).
  * \param[in] globalReadIndexPos   The $rglobal variable index on the controller used for the virtual motor's position readback value (RBV).
  * \param[in] globalWriteIndexPos  The $rglobal variable index on the controller used for the virtual motor's demanded position value (DMD).
  * \param[in] v_axisAddr 		    Index number of this axis, range 0 to pC->numAxes_-1.
  * \param[in] num_axes             Number of real motors associated with this co-ordinate system.
  * \param[in] r_axisList		    Indices or names of the real motors to which this co-ordinate system corresponds. Should be a comma separated string.

  */
Automation1CSAxis::Automation1CSAxis(Automation1MotorController* pC, const char* transformScriptName, const char* CS_Type, const char* CS_Name,
					 int globalReadIndexVel, int globalReadIndexPos, int globalWriteIndexPos, int v_axisAddr, int total_vAxes, const char* r_axisList )
    : Automation1MotorAxis(pC, v_axisAddr),
    pC_(pC),
    globalReadIndexVel_(globalReadIndexVel),
    globalReadIndexPos_(globalReadIndexPos),
    globalWriteIndexPos_(globalWriteIndexPos),
    transformScriptName_(transformScriptName),
    CS_Type_(CS_Type),
    CS_Name_(CS_Name),
    v_axisAddr_(v_axisAddr),
    total_vAxes_(total_vAxes),
    r_axisList_(r_axisList)
{
    Automation1_StatusConfig_Create(&(statusConfig_));

    // Gain Support is required for setClosedLoop to be called
    setIntegerParam(pC_->motorStatusGainSupport_, 1);

    std::vector<std::string> parsedAxisList = parseAxisList(r_axisList_, std::string(","));
    axisIndexList_.resize(parsedAxisList.size());                                                   // Resize this to have the same number of elements as axes.
    
    
    // In case we were given axis names rather than an index, try to match them up so we have an index list.    
    int availAxisCount = Automation1_Controller_AvailableAxisCount(pC_->controller_);
    int chrArrSz = 64;
    char axName[chrArrSz];
    bool mustWait;
    bool registrationClash;
    bool ret;
    
    for(int i=0; i<availAxisCount; i++)
    {               
        if(!Automation1_Parameter_GetAxisStringValue(pC_->controller_, i, Automation1AxisParameterId_AxisName, axName, chrArrSz))
        {
            logError("Unable to get axis name!");
        }

        for(uint j=0; j<parsedAxisList.size(); j++)
        {
            // If the axis name is NOT an int AND it matches an actual axis name then store its index else we were already given the index so copy it over.
            // Axis names are case sensitive.
            bool isint = isInteger(parsedAxisList[j]);
            if( !isint && (axName == parsedAxisList[j]) )
            {
                axisIndexList_[j] = i;
            }
            else if (isint)
            {
                axisIndexList_[j] = std::stoi(parsedAxisList[j].c_str());
            }
        } // for j
    } // for i
    
    
    // Add status items for all real motors associated with this virtual axis.
    this->numStatusItems_ = 0;
    for(uint i=0; i<axisIndexList_.size(); i++)
    {
        Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisIndexList_[i], Automation1AxisStatusItem_AxisStatus, 0);
        this->numStatusItems_++;
        Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisIndexList_[i], Automation1AxisStatusItem_DriveStatus, 0);
        this->numStatusItems_++;
        Automation1_StatusConfig_AddAxisStatusItem(statusConfig_, axisIndexList_[i], Automation1AxisStatusItem_AxisFault, 0);
        this->numStatusItems_++;
    }
   
    // Keep a list of all CS/virtual Axes created.
    csAxisList_.push_back( this );
    
    // Check if the transformation script is already running, if it isn't then start it.
    Automation1TaskStatus taskStatusArr[TRANSFORM_TASK_INDEX+1];
    ret = Automation1_Task_GetStatus(pC_->controller_, taskStatusArr, TRANSFORM_TASK_INDEX+1);
    
    
    if(!ret)
    {
        logError("ERROR: Could not get task status from controller.");
    }
    
    switch(taskStatusArr[TRANSFORM_TASK_INDEX].TaskState)
    {
        case Automation1TaskState_Unavailable:
            logError("ERROR: Transformation script cannot be run on chosen task! Task is unavailable.");
        break;
        
        case Automation1TaskState_Inactive:
            logError("ERROR: Transformation script cannot be run on chosen task! Task is disabled.");
        break;
        
        case Automation1TaskState_Idle:
            ret = Automation1_Task_ProgramRun(pC_->controller_, TRANSFORM_TASK_INDEX, transformScriptName_.c_str()); 
            if(!ret)
            {
                logError("ERROR: Could not start transformation script.");
            }
        break;
        
        case Automation1TaskState_Error:
            logError("ERROR: Transformation script cannot be run on chosen task! Task has an error.");
        break;
        // Other possibilities for TaskState not handled.
    }


    mustWait = true;

    // Register this CS/virtual axis with the controller script.
    // The number of checks here to prevent clashes between axes might be overkill but it runs through it pretty quickly anyway.
    while(mustWait)
    {

        // Scan through all CS Axes to see if any *other* CS Axis is trying to register.
        mustWait = false;
        for( uint i=0; i<csAxisList_.size(); i++)
        {
            if(csAxisList_[i] == this)                  {continue;}              // Only need to check *other* CS axis objects. Skip this one.
            if(csAxisList_[i]->registeringAxis == true)                          // Another CS Axis is currently registering itself so we must wait for it to complete.
            {
                mustWait = true;
                continue;
            }
        }
        
        // If no other CS Axis is registering then this CS Axis will signal it's attempting to otherwise it will restart the loop and check again.
        if(mustWait == false)                                                    // No registration is currently in progress so we can do this axis now.
        {
            this->registeringAxis = true;                                        // Set flag so other CS axes know we're using the controller globals.
        }
        else
        {
            this->registeringAxis = false;
            continue;                                                            // Restart the while loop to see if whichever axis was registering has finished.
        }
        
        // Check the axis list *again* to make sure there is *exactly one* CS Axis with the registering flag set (i.e. this one) otherwise unset the flag and restart the loop.
        // (In theory there might be several axes in the above loop at the same time. In practice these axes are created sequentially so this should never happen.)
        registrationClash = false;
        for( uint i=0; i<csAxisList_.size(); i++)
        {
            if(csAxisList_[i] == this)                  {continue;}              // Only need to check *other* CS axis objects. Skip this one.
            if(csAxisList_[i]->registeringAxis == true)                          // Another CS Axis is attempting to register itself at the same time.
            {
                registrationClash = true;                                        // Identify the clash and exit the for loop.
                break;
            }         
        }
        
        // If another CS Axis is trying to register at the same time then this axis should wait.
        if(registrationClash == true) 
        { 
            this->registeringAxis = false;
            mustWait = true;
            continue; 
        }
        else
        {
            this->registeringAxis = true;                                       // Probably no need to set it twice but let's make sure.
            
            // Set the various bits of info then set $dataWaiting flag for controller.    
            this->setScriptStringParam(this->CS_Name_.c_str(), RESERVED_SGLOBAL_CSNAME);
            this->setScriptStringParam(this->CS_Type_.c_str(), RESERVED_SGLOBAL_CSTYPE);
            this->setScriptStringParam(this->r_axisList_.c_str(), RESERVED_SGLOBAL_AXISLIST);
            this->setScriptIntegerParam(this->globalReadIndexVel_, RESERVED_IGLOBAL_RBV_VEL);
            this->setScriptIntegerParam(this->globalReadIndexPos_, RESERVED_IGLOBAL_RBV_POS);
            this->setScriptIntegerParam(this->globalWriteIndexPos_, RESERVED_IGLOBAL_DMD_POS);
            this->setDataWaitingState(DATA_WAITING_YES);          

            // Wait for DataWaiting to be set back to No before continuing.
            int dataWaitingState = -1;

            printf("Automation1 Axis %d: Waiting for controller to finish processing axis data...\n",v_axisAddr_);
            while(dataWaitingState != DATA_WAITING_NO)
            {
                dataWaitingState = getDataWaitingState();
            }
            
            this->registeringAxis = false;                                       // Unset registering flag and set registered flag.
            this->axisRegistered = true;
        } // if/else
    } // while  
    
    // Check over all of the CS axes to see if any still need to register.
    bool allDone = false;
    if(csAxisList_.size() == total_vAxes_)                                       // If csAxisList does not have total_vAxes_ elements then there are still CSaxes left to register.
    {
        allDone = true;
        for(uint i=0; i<csAxisList_.size(); i++)
        {
            if( csAxisList_[i]->axisRegistered == false )
            {
                allDone = false;
            }
        }   
     }  
     
    // If this is the last CS Axis to register then tell the controller there's no more data to wait for.
    if(allDone)
    {
        // Wait for $dataWaiting = DataWaiting.No before setting it to DataWaiting.Done
        int dataWaitingState = -1;

        printf("Automation1 Axis %d: Waiting for controller to finish processing final axis data...\n",v_axisAddr_);
        while(dataWaitingState != DATA_WAITING_NO)
        {
            dataWaitingState = getDataWaitingState();
        }

        printf("Automation1 Axis %d: Setting allDone.\n", v_axisAddr_);
        this->setDataWaitingState(DATA_WAITING_DONE);
    }       

    ctorComplete = true;
} // ctor


// Parses the string to determine if it contains only an integer.
// Returns 1/true if string can be converted to an int.
// Returns 0/false if string has other characters.
inline bool Automation1CSAxis::isInteger(const std::string & s)
{
   if(s.empty() || ((!isdigit(s[0])) && (s[0] != '-') && (s[0] != '+'))) return false;

   char * p;
   strtol(s.c_str(), &p, 10);

   return (*p == 0);
}


// Sets string global variables in the controller with the specified data.
void Automation1CSAxis::setScriptStringParam(const char* param, int global_index)
{
    bool ret;
    std::string setParamCmd = "$sglobal[" + std::to_string(global_index) + "]=\"" + param + "\"";
    ret = Automation1_Command_Execute(pC_->controller_, 1, setParamCmd.c_str());
    if(!ret)
    {
        logError("Error setting script string parameter!");
    }
}

// Sets integer global variables in the controller with the specified data.
void Automation1CSAxis::setScriptIntegerParam(int param, int global_index)
{
    bool ret;
    std::string setParamCmd = "$iglobal[" + std::to_string(global_index) + "]=" + std::to_string(param);
    ret = Automation1_Command_Execute(pC_->controller_, 1, setParamCmd.c_str());
    if(!ret)
    {
        logError("Error setting script integer parameter!");
    }
}

// Update DataWaiting status in controller script.
void Automation1CSAxis::setDataWaitingState(int waitState)
{
    bool ret;
    std::string setDataWaiting = "$iglobal[" + std::to_string(RESERVED_IGLOBAL_DATA) + "]=" + std::to_string(waitState);
    ret = Automation1_Command_Execute(pC_->controller_, 1, setDataWaiting.c_str());
    if(!ret)
    {
        logError("Error setting DataWaiting state");
    }
}

// Get DataWaiting status from controller script.
int Automation1CSAxis::getDataWaitingState(void)
{
    bool ret;
    int64_t dataWaitingState;
    std::string getDataWaiting_cmd = "$ireturn[0]=$iglobal[" + std::to_string(RESERVED_IGLOBAL_DATA) + "]";
    ret = Automation1_Command_ExecuteAndReturnAeroScriptInteger(pC_->controller_, 1, getDataWaiting_cmd.c_str(), &dataWaitingState);
    if(!ret)
    {
        logError("Error getting DataWaiting state");
    }
    return dataWaitingState;
}


// Parses a delimited list of axis names and/or indices from the boot script, separating them into a vector of a single axis per element.
// Delimiter is assumed to be a comma.
std::vector<std::string> Automation1CSAxis::parseAxisList( std::string axisList, std::string delimiter)
{
    size_t pos_start = 0;
    size_t pos_end;
    size_t delim_len = delimiter.length();
    std::string axisID;
    std::vector<std::string> parsedList;
    
    // Extract tokens based on delimiter.
    while( (pos_end = axisList.find(delimiter, pos_start)) != std::string::npos)
    {
        axisID = axisList.substr(pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        parsedList.push_back(axisID);
    }
    axisID = axisList.substr(pos_start, axisList.length()-pos_start);
    parsedList.push_back(axisID);
   
   return parsedList;
}

// Destructor.
Automation1CSAxis::~Automation1CSAxis()
{
    Automation1_StatusConfig_Destroy(statusConfig_);
}

extern "C" int Automation1CreateCSAxis(const char* portName, const char* transformScriptName, const char* CS_Type, 
    							const char* CS_Name, int globalReadIndexVel, int globalReadIndexPos, int globalWriteIndexPos, int v_axisAddr, int total_vAxes, const char* r_axisList)
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
    
    new Automation1CSAxis(pC, transformScriptName, CS_Type, CS_Name, globalReadIndexVel, globalReadIndexPos, globalWriteIndexPos, v_axisAddr, total_vAxes, r_axisList);
    
    return(asynSuccess);
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
    static const char *functionName = "Automation1CSAxis::move";
    double adjustedAcceleration = acceleration / countsPerUnitParam_;
    double adjustedVelocity = maxVelocity / countsPerUnitParam_;
    double adjustedPosition = position / countsPerUnitParam_;
    bool moveSuccessful;
    
    setIntegerParam(pC_->motorStatusProblem_,0);            // Unset "Problem" status bit. A logError call will set it.


    // Call move script here.
    std::string strWritePos = "$rglobal["+std::to_string(globalWriteIndexPos_)+"]="+ std::to_string(adjustedPosition);
	const char* cchrWritePos = strWritePos.c_str();

	if (!Automation1_Command_Execute(pC_->controller_, 1, cchrWritePos))
	{
		logError("ERROR: Could not set commanded position value to a controller $rglobal variable.");
	}

    // AeroScript call: CS_MoveAbsolute($CS_Name as string)
	std::string strMoveCmd = "CS_MoveAbsolute(\"" + CS_Name_ + "\")";
	moveSuccessful = Automation1_Command_Execute(pC_->controller_, 1, strMoveCmd.c_str());

    
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
    bool moveSuccessful;
    double adjustedAcceleration = acceleration / countsPerUnitParam_;
    double adjustedVelocity = maxVelocity / countsPerUnitParam_;
    
    setIntegerParam(pC_->motorStatusProblem_,0);            // Unset "Problem" status bit. A logError call will set it.

    for(uint i=0; i<axisIndexList_.size(); i++)
    {
        if (!Automation1_Command_SetupAxisRampValue(pC_->controller_,
                                                    1,
                                                    axisIndexList_[i],
                                                    Automation1RampMode_Rate,
                                                    adjustedAcceleration,
                                                    Automation1RampMode_Rate,
                                                    adjustedAcceleration))
        {
            logError("Failed to set acceleration of CS axes prior to moveVelocity (jog).");
            return asynError;
        }
    }
    
    // AeroScript call: CS_MoveVelocity($CS_Name as string, $velocity as real, $RBV_Index as integer)
    
    // The aeroscript library that receives this call uses the RBV global index (globalReadIndexPos_) to determine which virtual axis called it.
    // This is in case different coordinated movement is required on a per-axis basis.
    std::string strJogCmd = "CS_MoveVelocity(\"" + CS_Name_ + "\", " + std::to_string(adjustedVelocity) + "," + std::to_string(globalReadIndexPos_) +  ")";
	moveSuccessful = Automation1_Command_Execute(pC_->controller_, 1, strJogCmd.c_str());

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


/** Stop the real motors associated with this CS.
  * \param[in] acceleration The acceleration value. In the case of aborting motion in Automation1,
                            acceleration is determined by a separate controller parameter, so 
                            this is not used.
  */
asynStatus Automation1CSAxis::stop(double acceleration)
{
    setIntegerParam(pC_->motorStatusProblem_,0);            // Unset "Problem" status bit. A logError call will set it.
    for(uint i=0; i<axisIndexList_.size(); i++)
    {    
        if (!Automation1_Command_Abort(pC_->controller_, &axisIndexList_[i], 1))
        {
            logError("Failed to stop CS axes.");
            return asynError;
        }
    }
    return asynSuccess;
}


/** Moves all of the real motors associated with this CS to the home position.
  * \param[in] minVelocity  The initial velocity. Not used by Automation1.
  * \param[in] maxVelocity  The target velocity for the home. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec.
  * \param[in] forwards     Flag indicating motor direction. Not used by Automation1.
  */
asynStatus Automation1CSAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
    double adjustedAcceleration = acceleration / countsPerUnitParam_;
    
    setIntegerParam(pC_->motorStatusProblem_,0);            // Unset "Problem" status bit. A logError call will set it.
    for(uint i=0; i<axisIndexList_.size(); i++)
    {
        if (!Automation1_Command_SetupAxisRampValue(pC_->controller_,
                                                    1,
                                                    axisIndexList_[i],
                                                    Automation1RampMode_Rate,
                                                    adjustedAcceleration,
                                                    Automation1RampMode_Rate,
                                                    adjustedAcceleration))
        {
            logError("Failed to set acceleration prior to home.");
            return asynError;
        }
    }

    for(uint i=0; i<axisIndexList_.size(); i++)
    {    
        if (!Automation1_Command_HomeAsync(pC_->controller_, 1, &axisIndexList_[i], 1))
        {
            logError("Failed to home CS axes.");
            return asynError;
        }
    }
    
    return asynSuccess;
}


/** Enables or disables the real motors associated with this CS.
  * \param[in] closedLoop true = enable, false = disable.
  */
asynStatus Automation1CSAxis::setClosedLoop(bool closedLoop)
{
    if (closedLoop)
    {
        for(uint i=0; i<axisIndexList_.size(); i++)
        {
            if (!Automation1_Command_Enable(pC_->controller_, 1, &axisIndexList_[i], 1))
            {
                logError("Failed to enable CS axes.");
                return asynError;
            }
        }
    }
    else
    {
        for(uint i=0; i<axisIndexList_.size(); i++)
        {
            if (!Automation1_Command_Disable(pC_->controller_, &axisIndexList_[i], 1))
            {
                logError("Failed to disable CS axes.");
                return asynError;
            }
        }
    }
    setIntegerParam(pC_->motorStatusProblem_,0);	//Clear problem bit if it was set due to "axis not enabled"
    return asynSuccess;
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
    
// if(ctorComplete) blocks so that the polling thread doesn't start trying to use uninitialised variables whilst another thread is still in the constructor.
if(ctorComplete)
{
  
    double results[this->numStatusItems_];
        // [axisstatus, drivestatus, axisfault] per axis.
    int axisStatus[this->axisIndexList_.size()];
    int driveStatus[this->axisIndexList_.size()];
    int enabled;
    double positionRBV, positionDMD;
    double velocityRBV;
    double positionError;
    int axisFaults[this->axisIndexList_.size()];
    int done;

    int homed;
    int faults;
    
    double getMRES;

    std::string commandGetPos = "$rreturn[0] = $rglobal[" + std::to_string(globalReadIndexPos_) + "]";
    std::string commandGetVel = "$rreturn[0] = $rglobal[" + std::to_string(globalReadIndexVel_) + "]";
    std::string commandGetPosDMD = "$rreturn[0] = $rglobal[" + std::to_string(globalWriteIndexPos_) + "]";
    
    int numStatusItemsPerAxis = this->numStatusItems_/this->axisIndexList_.size();
    
    // This actually retrieves the status items from the controller. (Gets status for multiple real axes.)
    if (!Automation1_Status_GetResults(pC_->controller_,
                                       statusConfig_,
                                       results,
                                       (int)(sizeof(results) / sizeof(double))))
    {
        pollSuccessfull = false;
        goto skip;
    }
    

    for(uint i=0; i<this->axisIndexList_.size(); i++)
    {
        axisStatus[i]  = results[i*numStatusItemsPerAxis+0];   
        driveStatus[i] = results[i*numStatusItemsPerAxis+1];
        axisFaults[i]  = (int)results[i*numStatusItemsPerAxis+2];
    }
    
    // Get Position and Velocity RBVs from controller globals.
    
    if(!Automation1_Command_ExecuteAndReturnAeroScriptReal(pC_->controller_, 1, commandGetPos.c_str(), &positionRBV))
    {
        logError("Could not get position RBV.");
    }
    
    if(!Automation1_Command_ExecuteAndReturnAeroScriptReal(pC_->controller_, 1, commandGetVel.c_str(), &velocityRBV))
    {
        logError("Could not get velocity RBV.");
    }
    
    if(!Automation1_Command_ExecuteAndReturnAeroScriptReal(pC_->controller_, 1, commandGetPosDMD.c_str(), &positionDMD))
    {
        logError("Could not get position DMD.");
    }
    

    positionError = positionDMD - positionRBV;

    for(uint i=0; i<this->axisIndexList_.size(); i++)
    {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACEIO_DRIVER,
              "Automation1_Status_GetResults(%d): axis status = %d; drive status = %d; position feedback = %lf; velocity feedback %lf\n",
              axisNo_,
              axisStatus[i],
              driveStatus[i],
              positionRBV,
              velocityRBV);
    }
     
     
    // Set status bits as bitwise AND of component real axes. e.g. CS axes will not be enabled until *ALL* motors in the CS are enabled.         
    enabled = true;
    for(uint i=0; i<axisIndexList_.size(); i++)
    {   
        // If either of the real axes aren't enabled then don't report this axis as being enabled.
        if(!(driveStatus[i] & Automation1DriveStatus_Enabled))
        {
            enabled = false;
            break;
        }
    }
    

    // Since the virtual axis doesn't have a real motor this param is set according to whatever value of MRES was specified in the boot script.
    // These values being a little off can result in the motor position RBV also being a little off.
    pC_->getDoubleParam(v_axisAddr_,pC_->motorRecResolution_, &getMRES);
    countsPerUnitParam_ = 1.0 / getMRES;

    setIntegerParam(pC_->motorStatusPowerOn_, enabled);
    setDoubleParam(pC_->motorPosition_, positionRBV * countsPerUnitParam_);
    setDoubleParam(pC_->motorEncoderPosition_, positionRBV * countsPerUnitParam_);

    setDoubleParam(pC_->AUTOMATION1_C_Velocity_, velocityRBV);
    setDoubleParam(pC_->AUTOMATION1_C_FError_, positionError * countsPerUnitParam_);
    
    // ---------- MOTION DONE -----------
    done = true;
    for(uint i=0; i<axisIndexList_.size(); i++)
    {   
        if(!(axisStatus[i] & Automation1AxisStatus_MotionDone))
        {
            done = false;
            break;
        }
    }
    setIntegerParam(pC_->motorStatusDone_, done);
    
    // ---------- MOVING/DIRECTION -----------
    if (done)
    {
        *moving = false;
        setIntegerParam(pC_->motorStatusMoving_, 0);
    }
    else
    {
        if (velocityRBV != 0)
        {
            *moving = true;
            setIntegerParam(pC_->motorStatusMoving_, 1);
            if (velocityRBV > 0)
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
    
    // ---------- ANY FAULT -----------
    faults = 0;
    for(uint i=0; i<axisIndexList_.size(); i++)
    {   
        if(axisFaults[i] > 0)
        {
            faults = 1;
            break;
        }
    }
    setIntegerParam(pC_->motorStatusProblem_,faults);                               // Set "Problem" status bit if there are any axis faults.

    // ---------- HIGH LIMIT -----------
    for(uint i=0; i<axisIndexList_.size(); i++)
    {
        if ((axisFaults[i] & Automation1AxisFault_CwEndOfTravelLimitFault) ||
            (axisFaults[i] & Automation1AxisFault_CwSoftwareLimitFault))
        {
            setIntegerParam(pC_->motorStatusHighLimit_, 1);
            break;                                                                 // If there's a fault on either real axes then report a fault here.
        }
        else
        {
            setIntegerParam(pC_->motorStatusHighLimit_, 0);
        }
    }

    // ---------- LOW LIMIT -----------
    for(uint i=0; i<axisIndexList_.size(); i++)
    {
        if ((axisFaults[i] & Automation1AxisFault_CcwEndOfTravelLimitFault) ||
            (axisFaults[i] & Automation1AxisFault_CcwSoftwareLimitFault))
        {
            setIntegerParam(pC_->motorStatusLowLimit_, 1);
            break;
        }
        else
        {
            setIntegerParam(pC_->motorStatusLowLimit_, 0);
        }
    }
    
    // ---------- POSITION ERROR -----------
    for(uint i=0; i<axisIndexList_.size(); i++)
    {
        if (axisFaults[i] & Automation1AxisFault_PositionErrorFault)
        {
            setIntegerParam(pC_->motorStatusFollowingError_, 1);
            break;
        }
        else
        {
            setIntegerParam(pC_->motorStatusFollowingError_, 0);
        }
    }
    
    // ---------- HOMED -----------
    homed = true;
    for(uint i=0; i<axisIndexList_.size(); i++)
    {   
        if(!(axisStatus[i] & Automation1AxisStatus_Homed))
        {
            homed = false;
            break;
        }
    }
    setIntegerParam(pC_->motorStatusHomed_, homed);
}
skip:
    // ---------- COMMS ERROR -----------
    setIntegerParam(pC_->motorStatusCommsError_, !pollSuccessfull);
    callParamCallbacks();
    if (!pollSuccessfull)
    {
        logError("Poll failed.");
    }

    return pollSuccessfull ? asynSuccess : asynError;
} // poll


/** Function to define the motor positions for a profile move.
  * This function calls the base class method, then converts the positions to
  * controller units.
  * \param[in] positions Array of profile positions for this axis in user units.
  * \param[in] numPoints The number of positions in the array.
  */
asynStatus Automation1CSAxis::defineProfile(double* positions, size_t numPoints)
{
    double resolution;
    pC_->getDoubleParam(v_axisAddr_, pC_->motorRecResolution_, &resolution);
    // Call the base class function (converts from EGU to steps)
    asynStatus status = asynMotorAxis::defineProfile(positions, numPoints);
    if (status) return status;

    // Convert from steps to EGU.
    for (size_t i = 0; i < numPoints; i++)
    {
        profilePositions_[i] = profilePositions_[i] * resolution;
    }
    return asynSuccess;
}

asynStatus Automation1CSAxis::readbackProfile()
{
    // Call the base class method
    Automation1MotorAxis::readbackProfile();
    return asynSuccess;
}


/** Logs an driver error and error details from the C API.  Made to reduce duplicate code.
  * \param[in] driverMessage A char array meant to convey where in execution the error occured.
*/
void Automation1CSAxis::logError(const char* driverMessage)
{
    int errorCode = Automation1_GetLastError();
    char errorMessage[1024];
    Automation1_GetLastErrorMessage(errorMessage, 1024);
    
    setIntegerParam(pC_->motorStatusProblem_,1);            // Set "Problem" status bit for any failed API call including when a disabled axis is commanded.

    asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR,
              "Driver: Automation1. Function Message: %s. Axis: %d. API Error Code: %d. API Error Message: %s\n",
              driverMessage,
              axisNo_,
              errorCode,
              errorMessage);
    return;
}

// Code for iocsh registration
static const iocshArg Automation1CreateCSAxisArg0 = { "Port name",                                                      iocshArgString	};
static const iocshArg Automation1CreateCSAxisArg1 = { "Transformation Aeroscript name",                                 iocshArgString	};
static const iocshArg Automation1CreateCSAxisArg2 = { "Type of CS being created",                                       iocshArgString	};
static const iocshArg Automation1CreateCSAxisArg3 = { "Co-ordinate system name",                                        iocshArgString	};
static const iocshArg Automation1CreateCSAxisArg4 = { "Controller global variable for velocity reading",                iocshArgInt 	};
static const iocshArg Automation1CreateCSAxisArg5 = { "Controller global variable for position reading",                iocshArgInt 	};
static const iocshArg Automation1CreateCSAxisArg6 = { "Controller global variable for position writing",                iocshArgInt	    };
static const iocshArg Automation1CreateCSAxisArg7 = { "Asyn address on the controller this virtual axis will occupy",   iocshArgInt	    };
static const iocshArg Automation1CreateCSAxisArg8 = { "Total number of CS/virtual axes being created",                  iocshArgInt	    };
static const iocshArg Automation1CreateCSAxisArg9 = { "Real motors that this CS addresses",                             iocshArgString	};
static const iocshArg* const Automation1CreateCSAxisArgs[] = { &Automation1CreateCSAxisArg0,
                                                                   &Automation1CreateCSAxisArg1,
                                                                   &Automation1CreateCSAxisArg2,
                                                                   &Automation1CreateCSAxisArg3,
                                                                   &Automation1CreateCSAxisArg4,
                                                                   &Automation1CreateCSAxisArg5,
                                                                   &Automation1CreateCSAxisArg6,
                                                                   &Automation1CreateCSAxisArg7,
                                                                   &Automation1CreateCSAxisArg8,
                                                                   &Automation1CreateCSAxisArg9 };
static const iocshFuncDef Automation1CreateCSAxisDef = { "Automation1CreateCSAxis", 10  , Automation1CreateCSAxisArgs };
static void Automation1CreateCSAxisCallFunc(const iocshArgBuf* args)
{
    Automation1CreateCSAxis(args[0].sval, args[1].sval, args[2].sval, args[3].sval, 
    			     args[4].ival, args[5].ival, args[6].ival, args[7].ival, args[8].ival, args[9].sval);
}

static void Automation1CSRegister(void)
{
    iocshRegister(&Automation1CreateCSAxisDef, Automation1CreateCSAxisCallFunc);
}

extern "C" {
    epicsExportRegistrar(Automation1CSRegister);
}

