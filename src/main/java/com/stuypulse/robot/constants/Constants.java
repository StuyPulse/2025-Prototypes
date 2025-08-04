package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Constants{

    public interface Arm {
        
        Rotation2d SHOULDER_MIN_ANGLE = Rotation2d.fromDegrees(-90);
        Rotation2d SHOULDER_MAX_ANGLE = Rotation2d.fromDegrees(90);
        Rotation2d ELBOW_MIN_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d ELBOW_MAX_ANGLE = Rotation2d.fromDegrees(360);

        double SHOULDER_LENGTH = 0.9779;      // 38.5 in -> 0.9779 m
        double SHOULDER_MASS = 0.84277462346; // 1.858 lbs -> 0.84277462346 kg
        double SHOULDER_GEAR_RATIO = 3515/27; 
        // Everytime the gear rotates one, 4.625 of encoder rotates

        double ELBOW_LENGTH = 0.7112;    // 28 in -> 0.7112 m
        double ELBOW_MASS = 0.3991613;   // 0.88 lbs -> 0.3991613 kg
        double ELBOW_GEAR_RATIO = 54; // Double Check

        double BASE_HEIGHT = 0.2032; // 8 in -> 0.2032 m

    }

}

