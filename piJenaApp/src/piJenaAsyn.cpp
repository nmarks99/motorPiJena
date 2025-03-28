#include <asynOctetSyncIO.h>
#include <cstdio>
#include <cstdlib>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>

#include "piJenaAsyn.hpp"

constexpr int NUM_PARAMS = 0;

// actual resolution of sensor?
// MRES -> EGU
// 1.0  -> nanometers
// 1e-3 -> micrometers
// 1e-6 -> millimeters
// 1e-9 -> meters

// controller reports in microns, but has nanometer resolution (I think)
constexpr double DRIVER_RESOLUTION = 1000;
constexpr int DONE_TOLERANCE = 50; // done when within 50 microns of target
constexpr int CMD_BUFFER_SIZE = 30;

PiJenaMotorController::PiJenaMotorController(const char *portName, const char *PiJenaMotorPortName,
                                           int numAxes, double movingPollPeriod,
                                           double idlePollPeriod)
    : asynMotorController(portName, numAxes, NUM_PARAMS,
                          0, // No additional interfaces beyond the base class
                          0, // No additional callback interfaces beyond those in base class
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,    // autoconnect
                          0, 0) // Default priority and stack size
{
    asynStatus status;
    int axis;
    PiJenaMotorAxis *pAxis;
    static const char *functionName = "PiJenaMotorController::PiJenaMotorController";

    // Connect to motor controller
    status = pasynOctetSyncIO->connect(PiJenaMotorPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s: cannot connect to AKD2G controller\n",
                  functionName);
    }

    // Create PiJenaMotorAxis object for each axis
    // if not done here, user must call PiJenaMotorCreateAxis from cmd file
    for (axis = 0; axis < numAxes; axis++) {
        pAxis = new PiJenaMotorAxis(this, axis);
    }

    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

extern "C" int PiJenaMotorCreateController(const char *portName, const char *PiJenaMotorPortName,
                                          int numAxes, int movingPollPeriod, int idlePollPeriod) {
    PiJenaMotorController *pPiJenaMotorController = new PiJenaMotorController(
        portName, PiJenaMotorPortName, numAxes, movingPollPeriod / 1000., idlePollPeriod / 1000.);
    pPiJenaMotorController = NULL;
    return (asynSuccess);
}

void PiJenaMotorController::report(FILE *fp, int level) {
    // "dbior" from iocsh can be useful to see what's going on here
    fprintf(fp, "PiJena Motor Controller driver %s\n", this->portName);
    fprintf(fp, "    numAxes=%d\n", numAxes_);
    fprintf(fp, "    moving poll period=%f\n", movingPollPeriod_);
    fprintf(fp, "    idle poll period=%f\n", idlePollPeriod_);

    // Call the base class method
    asynMotorController::report(fp, level);
}

PiJenaMotorAxis *PiJenaMotorController::getAxis(asynUser *pasynUser) {
    return static_cast<PiJenaMotorAxis *>(asynMotorController::getAxis(pasynUser));
}

PiJenaMotorAxis *PiJenaMotorController::getAxis(int axisNo) {
    return static_cast<PiJenaMotorAxis *>(asynMotorController::getAxis(axisNo));
}

PiJenaMotorAxis::PiJenaMotorAxis(PiJenaMotorController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo), pC_(pC) {

    axisIndex_ = axisNo + 1;
    asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "PiJenaMotorAxis created with axis index %d\n",
              axisIndex_);

    // Gain Support is required for setClosedLoop to be called
    setIntegerParam(pC->motorStatusHasEncoder_, 1);
    setIntegerParam(pC->motorStatusGainSupport_, 1);

    callParamCallbacks();
}

void PiJenaMotorAxis::report(FILE *fp, int level) {
    if (level > 0) {
        fprintf(fp, " Axis #%d\n", axisNo_);
        fprintf(fp, " axisIndex_=%d\n", axisIndex_);
    }
    asynMotorAxis::report(fp, level);
}

asynStatus PiJenaMotorAxis::stop(double acceleration) {

    asynStatus asyn_status = asynSuccess;

    callParamCallbacks();
    return asyn_status;
}

asynStatus PiJenaMotorAxis::move(double position, int relative, double minVelocity,
                                double maxVelocity, double acceleration) {
    asynStatus asyn_status = asynStatus::asynSuccess;
   
    double pos_um = position / DRIVER_RESOLUTION;

    // printf("Move to %lf microns requested\n", pos_um);
    sprintf(pC_->outString_, "set,%d,%.3lf", axisNo_, pos_um);
    // printf("Writing '%s'\n", pC_->outString_);
    pC_->writeController(); // controller won't reply
    if (asyn_status) {
        callParamCallbacks();
        return asynError;
    }

    this->moveStarted_ = true;
    this->targetPos_ = position; // nanometers

    
    callParamCallbacks();
    return asyn_status;
}

asynStatus PiJenaMotorAxis::poll(bool *moving) {
    asynStatus asyn_status = asynSuccess;

    int position_nm = 0;
    auto last_comma_ind = std::string::npos; 

    sprintf(pC_->outString_, "mess,%d", axisNo_);
    asyn_status = pC_->writeReadController();

    // printf("Write '%s'\n", pC_->outString_);
    // printf("Read '%s'\n", pC_->inString_);

    std::string instr(pC_->inString_);
    last_comma_ind = instr.rfind(",");
    if (last_comma_ind != std::string::npos) {
        std::string rbk_str = instr.substr(last_comma_ind + 1);
        double position_um = std::stod(rbk_str);
        position_nm = position_um * DRIVER_RESOLUTION;    // convert to nanometers "steps"
        setDoubleParam(pC_->motorPosition_, position_nm); // RRBV [nanometers when MRES=1]
        setDoubleParam(pC_->motorEncoderPosition_, position_nm);
    } else {
        asyn_status = asynError;
    }

    if (moveStarted_) {
        
        // if still moving:
        // printf("Current position = %d nm\n", position_nm);
        // printf("Target position = %d nm\n", this->targetPos_);
        // printf("delta pos = %d\n\n", std::abs(position_nm - this->targetPos_));
        if (std::abs(position_nm - this->targetPos_) < DONE_TOLERANCE) {
            printf("Move complete\n");
            moveStarted_ = false;
            *moving = false;
        } else {
            *moving = true;
        }
    } else {
        *moving = false;
    }
    setIntegerParam(pC_->motorStatusDone_, not *moving);
    setIntegerParam(pC_->motorStatusMoving_, *moving);

    callParamCallbacks();
    return asyn_status;
}

asynStatus PiJenaMotorAxis::setClosedLoop(bool closedLoop) {
    asynStatus asyn_status = asynSuccess;

    callParamCallbacks();
    return asyn_status;
}

// ==================
// iosch registration
// ==================

static const iocshArg PiJenaMotorCreateControllerArg0 = {"Controller port name", iocshArgString};
static const iocshArg PiJenaMotorCreateControllerArg1 = {"asyn port name", iocshArgString};
static const iocshArg PiJenaMotorCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg PiJenaMotorCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg PiJenaMotorCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg *const PiJenaMotorCreateControllerArgs[] = {
    &PiJenaMotorCreateControllerArg0, &PiJenaMotorCreateControllerArg1,
    &PiJenaMotorCreateControllerArg2, &PiJenaMotorCreateControllerArg3,
    &PiJenaMotorCreateControllerArg4};
static const iocshFuncDef PiJenaMotorCreateControllerDef = {"PiJenaMotorCreateController", 5,
                                                           PiJenaMotorCreateControllerArgs};

static void PiJenaMotorCreateControllerCallFunc(const iocshArgBuf *args) {
    PiJenaMotorCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival,
                               args[4].ival);
}

static void PiJenaMotorRegister(void) {
    iocshRegister(&PiJenaMotorCreateControllerDef, PiJenaMotorCreateControllerCallFunc);
}

extern "C" {
epicsExportRegistrar(PiJenaMotorRegister);
}
