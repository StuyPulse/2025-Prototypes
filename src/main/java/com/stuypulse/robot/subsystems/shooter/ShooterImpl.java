package com.stuypulse.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

public class ShooterImpl extends Shooter {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    
    protected ShooterImpl() {
        super();
        this.leftMotor = new TalonFX(Ports.Shooter.LEFT_MOTOR);
        this.rightMotor = new TalonFX(Ports.Shooter.RIGHT_MOTOR);
        
        Motors.Shooter.SHOOTER_MOTOR_CONFIG.configure(leftMotor);
        Motors.Shooter.SHOOTER_MOTOR_CONFIG.configure(rightMotor);
    }

    @Override
    public void periodic() {
        super.periodic();
        
        leftMotor.set(getState().getSpeed());
        rightMotor.set(getState().getSpeed());
    }
}