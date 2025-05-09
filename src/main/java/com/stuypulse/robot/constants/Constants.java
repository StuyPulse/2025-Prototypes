package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public interface Constants{

    public interface Arm {
        double GEAR_RATIO = 86.0;

        Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(0);
        Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(360);
    }

}

