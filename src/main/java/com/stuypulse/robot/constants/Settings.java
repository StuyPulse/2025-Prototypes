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

    double DT = 0.02;

    public interface Turret {
        double TOLERANCE = 0.01;
        
        public interface States {
            Rotation2d INTAKE_SIDE = Rotation2d.fromDegrees(0);
            Rotation2d SHOOTER_SIDE = Rotation2d.fromDegrees(0);
        }
    }

    public interface HoodedShooter {
        double TOLERANCE = 0.01;

        public interface States {
            double CLOSE_ANGLE = 0;
            double FAR_ANGLE = 0;
            double FERRY_ANGLE = 0;
        }
    }
}
