package com.stuypulse.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.MathUtil;
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
        return MathUtil.clamp(getShooterSpeed().get(), 0.0, 1000.0);
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

        // set motion magic controls
        leftMotor.setControl(new VelocityVoltage(getTargetRPM() / 60.0));
        rightMotor.setControl(new VelocityVoltage(getTargetRPM() / 60.0));

        SmartDashboard.putNumber("Shooter/Left Motor Voltage", leftMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Right Motor Voltage", rightMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Left Motor Velocity (RPM)", 60.0 * leftMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/Right Motor Velocity (RPM)", 60.0 * rightMotor.getVelocity().getValueAsDouble());

        
    }
}