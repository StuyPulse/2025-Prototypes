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
        int HOOD_MOTOR = 10;
        int ROLLER_MOTOR = 0;
        int SHOOTER_MOTOR = 16;
    }

    public interface WiffleWorksShooter {
        int SHOOTER_MOTOR = 14;
    }

    //ALL PORTS ARE CORRECT - Ryan and co 11/5
    public interface Swerve {
        //int PIGEON = 9;
        public interface FrontRight {
            int DRIVE = 11; //(CHECKED)
            int TURN = 10; // (CHECKED)
            int ENCODER = 1;
        }
        public interface FrontLeft {
            int DRIVE = 13; // (CHECKED)
            int TURN = 12; // (CHECKED)
            int ENCODER = 2;
        }
        public interface BackLeft {
            int DRIVE = 15; // (CHECKED)
            int TURN = 14; // (CHECKED)
            int ENCODER = 3;
        }
        public interface BackRight {
            int DRIVE = 17; //(CHECKED)
            int TURN = 16; // (CHECKED)
            int ENCODER = 0;
        }
    }

    public interface Climber {
        int MOTOR_ONE = 40;
        int MOTOR_TWO = 2;
    }
}
