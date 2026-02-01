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
        double SHOOTER_SHOOT = 200;

        // magic motion const
        double SHOOTER_LEFT_MAX_VELOCITY = 1;
        double SHOOTER_RIGHT_MAX_VELOCITY = 1;

        double SHOOTER_LEFT_MAX_ACCEL = 1;
        double SHOTOER_RIGHT_MAX_ACCEL = 1;

        double TARGET_RPM_THRESHOLD = 1;
    }

    public interface Spindexer {
        SmartBoolean isSpindexerEnabled = new SmartBoolean("Spindexer/isEnabled? ", true);
        SmartBoolean debugMode = new SmartBoolean("Spindexer/ Debug Mode? ", false); //DEBUG MODE TOGGLE

        // SmartNumber krakenTargetSpeed = new SmartNumber("Spindexer/ Kraken/ Target Speed", 1.0);
        // SmartNumber neoTargetSpeed = new SmartNumber("Spindexer/ Neo/ Target Speed", 1.0);


        public SmartNumber SpindexerKrakenSpinSpeed = new SmartNumber("Spindexer/ Kraken/ Target Speed (CHANGEABLE)", 1.0); //0.2 Duty cycle value was 0.08003, 1.0 was also the same
        public double SpindexerKrakenStopSpeed = 0.0;

        public SmartNumber SpindexerNeoSpinSpeed = new SmartNumber("Spindexer/ Neo/ Target Speed (CHANGEABLE)", 1.0); 
        public double SpindexerNeoStopSpeed = 0.0;

    }
}
