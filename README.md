# Physical Computing 

## Intro
Code repository used to control the spherical manipulator and driven pendulum made for the Physical Computing module.

## Structure
This repo contains a PlatformIO project. When uploaded to an esp32, the main.cpp file is run.
All variable and function declarations are stored in the projectConfig.h file in gizmoPlatformio/lib
All functions run in the main.cpp file are stored in modules inside the gizmoPlatformio/src/modules directory.
Two custom libraries have been made for PID control and for control over the 2DOF spherical coaxial manipulator.

## Use
Two recreate this project...
