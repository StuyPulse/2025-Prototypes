/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {

    public enum RobotType {
        AUNT_MARY("00000"),
        SIM("");

        public final String serialNum;

        RobotType(String serialNum) {
            this.serialNum = serialNum;
        }

        public static RobotType fromString(String serialNum) {
            for (RobotType robot : RobotType.values()) {
                if (robot.serialNum.equals(serialNum.toUpperCase())) {
                    return robot;
                }
            }

            return RobotType.SIM;
        }
    }

    public interface Arm {
        public interface PID {
            double kP = 0.0;
            double kI = 0.0;
            double kD = 0.0;
        }
        public interface FF{
            double kS = 0.0;
            double kV = 0.0;
            double kA = 0.0;
            double kG = 0.0;
        }
        
        double MAX_VEL = 5.0;
        double MAX_ACCEL = 5.0;

        double L1_ANGLE = 30;
        double L2_ANGLE = 60;
        double L3_ANGLE = 90;
        double L4_ANGLE = 180;
        
        double LOWER_ANGLE_LIMIT = -180;
        double UPPER_ANGLE_LIMIT = 180;

        double GEAR_RATIO = 0.83333333333;
        double AREA = 3; // meters squared
        double MOMENT_OF_INERTIA = Units.lbsToKilograms(20) * AREA;
        double ARM_LENGTH = Units.inchesToMeters(3);

        double MAX_ANGLE_ERROR = 1;
    }
}

