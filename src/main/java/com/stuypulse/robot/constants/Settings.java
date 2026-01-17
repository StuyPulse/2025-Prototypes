/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    public interface Shooter {
        double SHOOTER_STOP = 0;
        double SHOOTER_SHOOT = 1;

        // magic motion const
        double SHOOTER_LEFT_MAX_VELOCITY = 1;
        double SHOOTER_RIGHT_MAX_VELOCITY = 1;

        double SHOOTER_LEFT_MAX_ACCEL = 1;
        double SHOTOER_RIGHT_MAX_ACCEL = 1;

        double TARGET_RPM_THRESHOLD = 1;
    }
}
