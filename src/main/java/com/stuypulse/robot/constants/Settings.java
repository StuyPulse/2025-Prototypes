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
    public interface Arm {
        double RESTING_SHOULDER = 0.0;
        double RESTING_ELBOW = 0.0;

        double L2_SHOULDER = 0.0;
        double L2_ELBOW = 0.0;

        double L3_SHOULDER = 0.0;
        double L3_ELBOW = 0.0;

        double L4_SHOULDER = 0.0;
        double L4_ELBOW = 0.0;

        Rotation2d SHOULDER_ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);
        Rotation2d ELBOW_ANGLE_OFFSET = Rotation2d.fromDegrees(0.0);

        public interface Shoulder{
            public interface PID{
                double kP = 0.0;
                double kI = 0.0;
                double kD = 0.0;
            }
            public interface FF{
                double kS = 0.0;
                double kG = 0.0;
                double kV = 0.0;
                double kA = 0.0;
            }
        }
        public interface Elbow{
            public interface PID{
                double kP = 0.0;
                double kI = 0.0;
                double kD = 0.0;
            }
            public interface FF{
                double kS = 0.0;
                double kG = 0.0;
                double kV = 0.0;
                double kA = 0.0;
            }
        }
    }
}


