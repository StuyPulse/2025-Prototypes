/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    double DT = 0.02;

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
