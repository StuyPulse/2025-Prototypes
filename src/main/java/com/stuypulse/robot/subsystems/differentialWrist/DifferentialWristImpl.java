package com.stuypulse.robot.subsystems.differentialWrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;


import edu.wpi.first.math.geometry.Rotation2d;

public class DifferentialWristImpl extends DifferentialWrist {
    private final TalonFX leftDifferentialMotor;
    private final TalonFX rightDifferentialMotor;
    private final TalonFX rollerMotor;

    public DifferentialWristImpl() {
        super();

        leftDifferentialMotor = new TalonFX(Ports.DifferentialWrist.LEFT_DIFFERENTIAL, "*");
        Motors.DifferentialWrist.LEFT_DIFFERENTIAL_MOTOR_CONFIG.configure(leftDifferentialMotor);

        rightDifferentialMotor = new TalonFX(Ports.DifferentialWrist.RIGHT_DIFFERENTIAL, "*");           
        Motors.DifferentialWrist.RIGHT_DIFFERENTIAL_MOTOR_CONFIG.configure(rightDifferentialMotor);

        rollerMotor = new TalonFX(Ports.DifferentialWrist.ROLLER, "*");
        Motors.DifferentialWrist.ROLLER_MOTOR_CONFIG.configure(rollerMotor);
    }

    @Override
    public Rotation2d getCurrentRollAngle() {
        return Rotation2d.fromRotations((leftDifferentialMotor.getPosition().getValueAsDouble() - rightDifferentialMotor.getPosition().getValueAsDouble()) / 2);
    }

    @Override
    public Rotation2d getCurrentPitchAngle() {
        return Rotation2d.fromRotations((leftDifferentialMotor.getPosition().getValueAsDouble() + rightDifferentialMotor.getPosition().getValueAsDouble()) / 2);
    }

}
