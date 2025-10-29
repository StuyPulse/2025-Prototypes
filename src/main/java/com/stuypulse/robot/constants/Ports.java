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
    public interface HDSR {
        int HOOD_MOTOR = 0;
        int ROLLER_MOTOR = 1;
        int SHOOTER_MOTOR = 14;
    }

    //TODO: Fix all these ports, they are wrong
    public interface Swerve {
        int PIGEON = 9;
        public interface FrontRight {
            int DRIVE = 10;
            int TURN = 17;
            int ENCODER = 1;
        }
        public interface FrontLeft {
            int DRIVE = 18;
            int TURN = 15;
            int ENCODER = 4;
        }
        public interface BackLeft {
            int DRIVE = 0;
            int TURN = 13;
            int ENCODER = 3;
        }
        public interface BackRight {
            int DRIVE = 16;
            int TURN = 12;
            int ENCODER = 2;
        }
    }
}
