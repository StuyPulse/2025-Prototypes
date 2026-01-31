/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Spindexer {
        public interface Roller {
            SmartNumber ROLLER_SPEED = new SmartNumber("Subsystems/Spindexer/Roller/Roller Speed", 1.0);
        }
        public interface Spinner {
            SmartNumber SPINNER_SPEED = new SmartNumber("Subsystems/Spindexer/Spinner/Spinner Speed", 1.0);
        }
    }
}
