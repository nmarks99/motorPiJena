#include "asynDriver.h"
#include "asynMotorAxis.h"
#include "asynMotorController.h"

class epicsShareClass PiJenaMotorAxis : public asynMotorAxis {
  public:
    PiJenaMotorAxis(class PiJenaMotorController *pC, int axisNo);
    void report(FILE *fp, int level);
    asynStatus stop(double acceleration);
    asynStatus poll(bool *moving);
    asynStatus setClosedLoop(bool closedLoop);
    asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);

  private:
    PiJenaMotorController *pC_;
    int axisIndex_;
    bool moveStarted_ = false;
    int targetPos_; // nanometers

    friend class PiJenaMotorController;
};

class epicsShareClass PiJenaMotorController : public asynMotorController {
  public:
    /// \brief Create a new PiJenaMotorController object
    ///
    /// \param[in] portName             The name of the asyn port that will be created for this
    /// driver
    /// \param[in] PiJenaPortName        The name of the drvAsynIPPort that was created previously
    /// \param[in] numAxes              The number of axes that this controller supports
    /// \param[in] movingPollPeriod     The time between polls when any axis is moving
    /// \param[in] idlePollPeriod       The time between polls when no axis is moving
    PiJenaMotorController(const char *portName, const char *PiJenaMotorController, int numAxes,
                         double movingPollPeriod, double idlePollPeriod);
    void report(FILE *fp, int level);

    /// \brief Returns a pointer to a PiJenaMotorAxis object
    /// \param[in] asynUser structure that encodes the axis index number
    /// \returns NULL if the axis number encoded in pasynUser is invalid
    PiJenaMotorAxis *getAxis(asynUser *pasynUser);

    /// \brief Returns a pointer to a PiJenaMotorAxis object
    /// \param[in] axisNo Axis index number
    /// \returns NULL if the axis number is invalid
    PiJenaMotorAxis *getAxis(int axisNo);

    // protected:
    // integer asyn param indices defined here

    friend class PiJenaMotorAxis;
};
