package com.stuypulse.robot.subsystems.superStructure.arm;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.hardware.TalonFX;

public class ArmImpl extends Arm {
    private TalonFX motor;

    public ArmImpl() {
        motor = new TalonFX(Ports.Arm.MOTOR);
        Motors.Arm.MOTOR_CONFIG.configure(motor); 
        motor.setPosition(Settings.Arm.MIN_ANGLE.getRotations());
    }

    private boolean isWithinTolerance(Rotation2d tolerance) {
        return Math.abs(getCurrentAngle().getDegrees() - getTargetAngle().getDegrees()) < tolerance.getDegrees();
    }

    private Rotation2d getTargetAngle() {
        return Rotation2d.fromDegrees(SLMath.clamp(getState().getTargetAngle().getDegrees(), Settings.Arm.MIN_ANGLE.getDegrees(), Settings.Arm.MAX_ANGLE.getDegrees()));
    }

    @Override
    public boolean atTargetAngle() {
        return isWithinTolerance(Settings.Arm.ANGLE_TOLERANCE);
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(motor.getPosition().getValueAsDouble());
    }
    
}