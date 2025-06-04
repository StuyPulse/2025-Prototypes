package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public interface Constants{

    public interface Arm {
        double GEAR_RATIO = 86.0;
        

        Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(360); // Need to change

        double SHOULDER_LENGTH = 0.5;
        double SHOULDER_MASS = 0.1;
        double SHOULDER_GEAR_RATIO = 85; // Double Check
        

        double ELBOW_LENGTH = 0.4;
        double ELBOW_MASS = 0.1;
        double ELBOW_GEAR_RATIO = 54; // Couble Check

        double BASE_HEIGHT = 0.2;

    }

}

