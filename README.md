# motorPiJena
EPICS motor drivers for the following [Piezosystem Jena](https://www.piezosystem.com/) controllers: EDS data interface module

[![Build Status](https://github.com/epics-motor/motorPiJena/actions/workflows/ci-scripts-build.yml/badge.svg)](https://github.com/epics-motor/motorPiJena/actions/workflows/ci-scripts-build.yml)
<!--[![Build Status](https://travis-ci.org/epics-motor/motorPiJena.png)](https://travis-ci.org/epics-motor/motorPiJena)-->

motorPiJena is a submodule of [motor](https://github.com/epics-modules/motor).  When motorPiJena is built in the ``motor/modules`` directory, no manual configuration is needed.

motorPiJena can also be built outside of motor by copying it's ``EXAMPLE_RELEASE.local`` file to ``RELEASE.local`` and defining the paths to ``MOTOR`` and itself.

motorPiJena contains an example IOC that is built if ``CONFIG_SITE.local`` sets ``BUILD_IOCS = YES``.  The example IOC can be built outside of driver module.
