package com.stuypulse.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterImpl extends Shooter {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    protected ShooterImpl() {
        super();
        this.leftMotor = new TalonFX(Ports.Shooter.LEFT_MOTOR);
        this.rightMotor = new TalonFX(Ports.Shooter.RIGHT_MOTOR);

        Motors.Shooter.SHOOTER_MOTOR_LEFT_CONFIG.configure(leftMotor);
        Motors.Shooter.SHOOTER_MOTOR_RIGHT_CONFIG.configure(rightMotor);
    }

    public double getTargetRPM() {
        return getShooterSpeed().get();
    }

    public boolean atTargetSpeeds() {
        return Math.abs(getCurrentLeftRPM() - getTargetRPM()) < Settings.Shooter.TARGET_RPM_THRESHOLD
                && Math.abs(getCurrentRightRPM() - getTargetRPM()) < Settings.Shooter.TARGET_RPM_THRESHOLD;
    }

    public double getCurrentLeftRPM() {
        return leftMotor.getVelocity().getValueAsDouble();
    }

    public double getCurrentRightRPM() {
        return rightMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putString("Shooter/ShooterState", getState().toString());
        // set motion magic controls
        leftMotor.setControl(new MotionMagicVelocityVoltage(getTargetRPM()));
        rightMotor.setControl(new MotionMagicVelocityVoltage(getTargetRPM()));
    }
}