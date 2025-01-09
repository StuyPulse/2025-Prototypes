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

    public interface VFourBar {
        public interface PID {
            double KP = 0.0;
            double KI = 0.0;
            double KD = 0.0;
        }

        double L1_ANGLE = 0;
        double L2_ANGLE = 0;
        double L3_ANGLE = 0;
        double L4_ANGLE = 0;
        
        double LOWER_ANGLE_LIMIT = 0;
        double UPPER_ANGLE_LIMIT = 90;

        double DEG_TO_RAD = 2*Math.PI/360;

        double GEAR_RATIO = 0.83333333333;
        double AREA = 3; // meters squared
        double MOMENT_OF_INERTIA = Units.lbsToKilograms(20) * AREA;
        double ARM_LENGTH = Units.inchesToMeters(3);


    }
}
