/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public interface Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }

    public interface DifferentialWrist {
        int LEFT_DIFFERENTIAL_MOTOR = 0;
        int RIGHT_DIFFERENTIAL_MOTOR = 0;
        int ROLLER_MOTOR = 0;

        int LEFT_ENCODER = 0;
        int RIGHT_ENCODER = 0;
    }
}
