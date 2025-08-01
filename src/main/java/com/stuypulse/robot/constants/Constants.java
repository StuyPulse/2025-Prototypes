package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public interface Constants{

    public interface Arm {
        // Change
        double GEAR_RATIO = 86.0;
        
        // Change
        Rotation2d SHOULDER_MIN_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d SHOULDER_MAX_ANGLE = Rotation2d.fromDegrees(360);
        Rotation2d ELBOW_MIN_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d ELBOW_MAX_ANGLE = Rotation2d.fromDegrees(360);

        double SHOULDER_LENGTH = 0.5; // m
        double SHOULDER_MASS = 0.1;   // kg
        double SHOULDER_GEAR_RATIO = 85; // Double Check
        

        double ELBOW_LENGTH = 0.4; // m
        double ELBOW_MASS = 0.1;   // kg
        double ELBOW_GEAR_RATIO = 54; // Couble Check

        double BASE_HEIGHT = 0.2;

    }

}

