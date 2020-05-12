# motorPiJena Releases

## __R1-0-1 (2020-05-12)__
R1-0-1 is a release based on the master branch.  

### Changes since R1-0

#### New features
* None

#### Modifications to existing features
* None

#### Bug fixes
* Commit [733d9b0](https://github.com/epics-motor/motorPiJena/commit/733d9b0e6beba9e891487bf81fe05d5fd367c490): Include ``$(MOTOR)/modules/RELEASE.$(EPICS_HOST_ARCH).local`` instead of ``$(MOTOR)/configure/RELEASE``
* Pull request [#1](https://github.com/epics-motor/motorPiJena/pull/1): Eliminated compiler warnings

## __R1-0 (2019-04-18)__
R1-0 is a release based on the master branch.  

### Changes since motor-6-11

motorPiJena is now a standalone module, as well as a submodule of [motor](https://github.com/epics-modules/motor)

#### New features
* motorPiJena can be built outside of the motor directory
* motorPiJena has a dedicated example IOC that can be built outside of motorPiJena

#### Modifications to existing features
* None

#### Bug fixes
* None
