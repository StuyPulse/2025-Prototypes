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

    public interface Telescope {
        double L1_HEIGHT_METERS = 0;
        double L2_FRONT_HEIGHT_METERS = 0;
        double L3_FRONT_HEIGHT_METERS = 0;
        double L4_FRONT_HEIGHT_METERS = 0;

        double L1_BACK_HEIGHT_METERS = 0;
        double L2_BACK_HEIGHT_METERS = 0;
        double L3_BACK_HEIGHT_METERS = 0;
        double L4_BACK_HEIGHT_METERS = 0;
        double kG_min = 0;
        double kG_max = 0;

        //constraints
        double MIN_HEIGHT_METERS = 0;
        double HEIGHT_TOLERANCE_METERS = 0;
        double MAX_ACCELERATION = 0;
        double MAX_VELOCITY = 0;

        public interface PID{
            double kP = 1.0;
            double kI = 1.0;
            double kD = 1.0;
            }

        public interface FeedForward{
            double kS = 0.1;
            double kV = 0.1;
            double kA = 0.1;
            double kG = 0.1;
        }
    }

    public interface Arm {
        Rotation2d L1_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d L2_ANGLE_FRONT = Rotation2d.fromDegrees(0);
        Rotation2d L3_ANGLE_FRONT = Rotation2d.fromDegrees(0);
        Rotation2d L4_ANGLE_FRONT = Rotation2d.fromDegrees(0);
        Rotation2d L2_ANGLE_BACK = Rotation2d.fromDegrees(0);
        Rotation2d L3_ANGLE_BACK = Rotation2d.fromDegrees(0);
        Rotation2d L4_ANGLE_BACK = Rotation2d.fromDegrees(0);

        //constraints
        Rotation2d ANGLE_TOLERANCE = Rotation2d.fromDegrees(0);
        Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(0);

        double MAX_ACCELERATION = 0;
        double MAX_VELOCITY = 0;

        public interface FeedForward {
            double kS = 0.1;
            double kV = 0.1;
            double kA = 0.1;
            double kG = 0.1;
        }

        public interface PID {
            double kP = 1.0;
            double kI = 1.0;
            double kD = 1.0;
        }
    }

}
