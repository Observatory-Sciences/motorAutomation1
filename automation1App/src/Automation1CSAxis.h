#ifndef Automation1CSAxis_H
#define Automation1CSAxis_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "Automation1MotorAxis.h"
#include "Include/Automation1.h"
#include <vector>

#define TRANSFORM_TASK_INDEX 3

// The following integer and string global indices on the controller should not be used for other purposes.
// Additionally the definitions should match those contained in the "CS_Defines.ascript" controller file.
#define RESERVED_IGLOBAL_RBV_VEL		0
#define RESERVED_IGLOBAL_RBV_POS		1
#define RESERVED_IGLOBAL_DMD_POS		2
#define RESERVED_IGLOBAL_DATA		    3
#define RESERVED_SGLOBAL_CSNAME	        0
#define RESERVED_SGLOBAL_AXISLIST	    1

// Could probably be done with an enum..
#define DATA_WAITING_NO		    0
#define DATA_WAITING_YES		1
#define DATA_WAITING_DONE		2

class Automation1MotorController;
class Automation1MotorAxis;

class epicsShareClass Automation1CSAxis : public Automation1MotorAxis
{
public:
    // Member functions we override from the base class.
    Automation1CSAxis(Automation1MotorController* pC, const char* transformScriptName, const char* moveScriptName, 
    							const char* CS_Name, int globalReadIndexVel, int globalReadIndexPos, int globalWriteIndexPos, int v_axisAddr, int total_vAxes, const char* r_axisList );
    ~Automation1CSAxis();

    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus moveVelocity(double min_velocity, double max_velocity, double acceleration);
    asynStatus poll(bool* moving);



private:
    
    void setScriptStringParam(const char* param, int global_index);                                     // Sets a string parameter in the Aeroscript transformation.
    void setScriptIntegerParam(int param, int global_index);                                            // Sets an integer parameter in the Aeroscript transformation.
    void setDataWaitingState(int waitState);                                                            // Sets the data waiting state in Aeroscript transformation.
    int getDataWaitingState(void);                                                                      // Retrieves data waiting state from Aeroscript transformation.
    std::vector<std::string> parseAxisList( std::string axisList, std::string delimiter);               // Parses a delimited list of axes into a vector.
    inline bool isInteger(const std::string & s);                                                       // Determines if a string contains only integers or not.
    
    Automation1MotorController* pC_;                                                                    // Pointer to asynMotorController to which the axis belongs.
    

    //TODO: Tidy this up and describe all vars.
    
    std::vector<int> axisIndexList_;                                                                    // Contains a list of the indices of real axes associated with this CS.

    int globalReadIndexPos_, globalWriteIndexPos_, globalReadIndexVel_;                                 // $iglobal index on controller to use for POS_RBV, POS_DMD and VEL_RBV.
    
    int numStatusItems_;                                                                                // Total number of status items being tracked by this virtual axis.
    
    const char* transformScriptName_, *moveScriptName_;                                                 // Aeroscript name to use for transformations and moves.
    const char* CS_Name_;                                                                               // Name of the CS that this virtual axis belongs to.
    
    int v_axisAddr_, total_vAxes_;                                                                      // The asyn address this virtual axis will occupy in the controller and 
                                                                                                        // total number of virtual axes in this CS.
    const char* r_axisList_;                                                                            // A list of the real motors that the co-ordinate system this axis belongs to will use. 
    
    Automation1StatusConfig statusConfig_;                                                              // The config is used by certain functions in the Automation1 C API to
                                                                                                        // get status items from the controller.  This is needed for polling.
    double countsPerUnitParam_;
    
    static std::vector< Automation1CSAxis* > csAxisList_;                                               // Contains a list of all virtual axes.

    bool registeringAxis = false;			                                                            // Set to true whilst registering is in progress then reset to false.
    bool axisRegistered = false;			                                                            // Set to true when axis has been registered.

    // Automation1 error codes and messages must be acquired through
    // calls to the C API.  To avoid duplicate code, we wrap calls 
    // to those functions and to asynPrint in this function.
    void logError(const char* driverMessage);

    friend class Automation1MotorController;
};

std::vector<Automation1CSAxis*> Automation1CSAxis::csAxisList_;

#endif
