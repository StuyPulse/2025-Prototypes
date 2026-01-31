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

    public interface Shooter{
        int LEFT_MOTOR = 3;
        int RIGHT_MOTOR = 4;
    }

    public interface Spindexer {
        int SPINDEXER_KRAKEN_MOTOR = 1; //TODO: fill in
        int SPINDEXER_NEO_MOTOR = 2; //TODO: fill in
    }
}
