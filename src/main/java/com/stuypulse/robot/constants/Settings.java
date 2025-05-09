/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {


    public interface DifferentialWrist{

        double PITCH_ANGLE_TOLERANCE = 0;
        double ROLL_ANGLE_TOLERANCE = 0;

        double MIN_PITCH_ANGLE = 0;
        double MAX_PITCH_ANGLE = 0;
        double MIN_ROLL_ANGLE = 0;
        double MAX_ROLL_ANGLE = 0;

        double LEFT_ANGLE_OFFSET = 0;
        double RIGHT_ANGLE_OFFSET = 0;
    }
}
