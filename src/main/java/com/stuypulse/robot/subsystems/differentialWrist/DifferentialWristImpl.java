package com.stuypulse.robot.subsystems.differentialWrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.math.geometry.Rotation2d;

public class DifferentialWristImpl extends DifferentialWrist {
    private final TalonFX leftDifferentialMotor;
    private final TalonFX rightDifferentialMotor;
    private final TalonFX rollerMotor;

    public DifferentialWristImpl() {
        /*
         * Configure motors after Motors.java has been updated.
         */

        leftDifferentialMotor = new TalonFX(Ports.DifferentialWrist.LEFT_DIFFERENTIAL, "*");
        
        rightDifferentialMotor = new TalonFX(Ports.DifferentialWrist.RIGHT_DIFFERENTIAL, "*");

        rollerMotor = new TalonFX(Ports.DifferentialWrist.ROLLER, "*");
    }

    @Override
    public Rotation2d getCurrentRollAngle() {
        return Rotation2d.fromRotations(leftDifferentialMotor.getPosition().getValueAsDouble() - rightDifferentialMotor.getPosition().getValueAsDouble());
    }

    @Override
    public Rotation2d getCurrentPitchAngle() {
        return Rotation2d.fromRotations(leftDifferentialMotor.getPosition().getValueAsDouble() - rightDifferentialMotor.getPosition().getValueAsDouble());
    }
}
