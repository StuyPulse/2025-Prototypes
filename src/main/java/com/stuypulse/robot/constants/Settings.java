/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.constants;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
    double DT = 0.020;
    public interface WiffleWorksShooter {
        
        public static final double ROLLER_SPEED = 1.0;
        public static final double MAX_DISTANCE_METERS = 20;
        public static final double MIN_DISTANCE_METERS = 5;

        //Shooter to distance  
        public static final Translation2d[] distanceXrpm = {new Translation2d(5, 1000 ), new Translation2d(10, 2000), new Translation2d(15, 3000 ), new Translation2d(20, 4000 )};
        public static final double xRPMSlope = 1;
        public static final double xRPMB = 1;

    }

    public interface HDSR {
        public interface Roller {
            public static final double intakeSpeed = 0; 
            public static final double stowSpeed = 0;
            public static final double ShootSpeed = 0;
        }
        
    }

    public interface Swerve {
        String DRIVE_CANBUS = "Swerve Drive Drive";
        double MODULE_VELOCITY_DEADBAND = 0.05;

        public interface Constraints {
            double MAX_MODULE_SPEED = 4.9;

            SmartNumber MAX_VELOCITY = new SmartNumber("Swerve/Motion/Max Velocity (m per s)", 3.0);
            SmartNumber MAX_ACCELERATION = new SmartNumber("Swerve/Motion/Max Acceleration (m per s^2)", 5.0);
            SmartNumber MAX_ANGULAR_VELOCITY = new SmartNumber("Swerve/Motion/Max Angular Velocity (rad per s)", Units.degreesToRadians(360));
            SmartNumber MAX_ANGULAR_ACCELERATION = new SmartNumber("Swerve/Motion/Max Angular Acceleration (rad per s^2)", Units.degreesToRadians(720));

            PathConstraints DEFAULT_CONSTRAINTS =
                new PathConstraints(
                    MAX_VELOCITY.get(),
                    MAX_ACCELERATION.get(),
                    MAX_ANGULAR_VELOCITY.get(),
                    MAX_ANGULAR_ACCELERATION.get());
        }
        public interface Alignment {
            PIDConstants XY = new PIDConstants(2.0, 0, 0.02);
            PIDConstants THETA = new PIDConstants(3, 0, 0.1);

            SmartNumber X_TOLERANCE = new SmartNumber("Alignment/X Tolerance (m)", 0.05);
            SmartNumber Y_TOLERANCE = new SmartNumber("Alignment/Y Tolerance (m)", 0.05);
            SmartNumber THETA_TOLERANCE = new SmartNumber("Alignment/Theta Tolerance (rad)", 0.1);

            double ALIGNMENT_DEBOUNCE = 0.05;
        }
        public interface Turn {
            double kP = 2.0;
            double kI = 0.0;
            double kD = 0.05;
        }

        public interface Drive {
            double kP = 0.5;
            double kI = 0.0;
            double kD = 0.01;

            double kS = 0.26722;
            double kV = 2.2119;
            double kA = 0.36249;
        }
    }

    public interface Driver {
        public interface Drive {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Drive/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Drive/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Drive/Power", 2);

            SmartNumber MAX_TELEOP_SPEED = new SmartNumber("Driver Settings/Drive/Max Speed", Swerve.Constraints.MAX_VELOCITY.get());
            SmartNumber MAX_TELEOP_ACCEL = new SmartNumber("Driver Settings/Drive/Max Accleration", Swerve.Constraints.MAX_ACCELERATION.get());
        }

        public interface Turn {
            SmartNumber DEADBAND = new SmartNumber("Driver Settings/Turn/Deadband", 0.05);

            SmartNumber RC = new SmartNumber("Driver Settings/Turn/RC", 0.05);
            SmartNumber POWER = new SmartNumber("Driver Settings/Turn/Power", 2);

            SmartNumber MAX_TELEOP_TURN_SPEED = new SmartNumber("Driver Settings/Turn/Max Turn Speed (rad per s)", Swerve.Constraints.MAX_ANGULAR_VELOCITY.get());
            SmartNumber MAX_TELEOP_TURN_ACCEL = new SmartNumber("Driver Settings/Turn/Max Turn Accel (rad per s^2)", Swerve.Constraints.MAX_ANGULAR_ACCELERATION.get());
        }
    }


    
}
