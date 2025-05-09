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

    public interface Telescope {
        double L1_HEIGHT_METERS = 0;
        double L2_FRONT_HEIGHT_METERS = 0;
        double L3_FRONT_HEIGHT_METERS = 0;
        double L4_FRONT_HEIGHT_METERS = 0;

        double L1_BACK_HEIGHT_METERS = 0;
        double L2_BACK_HEIGHT_METERS = 0;
        double L3_BACK_HEIGHT_METERS = 0;
        double L4_BACK_HEIGHT_METERS = 0;

    }

    public interface Arm {
        Rotation2d L1_ANGLE = 0;
        Rotation2D L2_ANGLE_FRONT = 0;
        Rotation2d L3_ANGLE_FRONT = 0;
        Rotation2d L4_ANGLE_FRONT = 0;
        Rotation2d L2_ANGLE_BACK = 0;
        Rotation2d L3_ANGLE_BACK = 0;
        Rotation2d L4_ANGLE_BACK = 0;
        
    }

}
