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

        double PITCH_ANGLE_TOLERANCE = 0.1;
        double ROLL_ANGLE_TOLERANCE = 0.1;

        double MIN_PITCH_ANGLE = 0;
        double MAX_PITCH_ANGLE = 180;
        double MIN_ROLL_ANGLE = 0;
        double MAX_ROLL_ANGLE = 180;


        double LEFT_ANGLE_OFFSET = 0;
        double RIGHT_ANGLE_OFFSET = 0;


        double STOW_PITCH_ANGLE = 0;
        double STOW_ROLL_ANGLE = 0;

        double CORAL_SCORE_L1_PITCH_ANGLE = 90;
        double CORAL_SCORE_L1_ROLL_ANGLE = 0;

        double CORAL_SCORE_L2_PITCH_ANGLE = 0;
        double CORAL_SCORE_L2_ROLL_ANGLE = 0;

        double CORAL_SCORE_L3_PITCH_ANGLE = 0;
        double CORAL_SCORE_L3_ROLL_ANGLE = 0;

        double CORAL_SCORE_L4_PITCH_ANGLE = 0;
        double CORAL_SCORE_L4_ROLL_ANGLE = 0;

        double ALGAE_SCORE_BARGE_PITCH_ANGLE = 0;
        double ALGAE_SCORE_BARGE_ROLL_ANGLE = 0;
        
        double ALGAE_SCORE_PROCESSOR_PITCH_ANGLE = 0;
        double ALGAE_SCORE_PROCESSOR_ROLL_ANGLE = 0;
        
        
    }

}
