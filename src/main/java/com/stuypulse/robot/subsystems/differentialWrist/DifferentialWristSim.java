package com.stuypulse.robot.subsystems.differentialWrist;

import edu.wpi.first.math.geometry.Rotation2d;

public class DifferentialWristSim extends DifferentialWrist {
    @Override
    public Rotation2d getCurrentRollAngle() {
        return null;
    }

    @Override
    public Rotation2d getCurrentPitchAngle() {
        return null;
    }

    @Override
    public boolean isAtTargetPitchAngle() {
        return false;
    }

    @Override
    public boolean isAtTargetRollAngle() {
        return false;
    }
}
