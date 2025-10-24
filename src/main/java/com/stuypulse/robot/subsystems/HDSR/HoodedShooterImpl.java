package com.stuypulse.robot.subsystems.HDSR;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

public class HoodedShooterImpl extends HoodedShooter {
    // private final TalonFX hoodMotor;
    // private final TalonFX rollerMotor;
    private final TalonFX shooterMotor;
    
    private boolean hasBall;

    public HoodedShooterImpl() {
        super();

        // hoodMotor = new TalonFX(Ports.HDSR.HOOD_MOTOR);
        // rollerMotor = new TalonFX(Ports.HDSR.ROLLER_MOTOR);
        shooterMotor = new TalonFX(Ports.HDSR.SHOOTER_MOTOR);
        
        hasBall = false;
        
        Motors.SHOOTER_MOTOR_CONFIG.configure(shooterMotor);
        // Motors.ROLLER_MOTOR_CONFIG.configure(rollerMotor);
        // Motors.HOOD_MOTOR_CONFIG.configure(hoodMotor);
    }

    // public Rotation2d getCurrentAngle() {
    //     return Rotation2d.fromRotations(hoodMotor.getPosition().getValueAsDouble());
    // }

    public double getCurrentVelocity() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    // public void setRollerSpeeds(double speed){
    //     rollerMotor.set(speed);
    // }
   
    @Override
    public void periodic() {
        super.periodic();

        // hoodMotor.setControl(new PositionVoltage(getState().getTargetAngle().getRotations()));
        shooterMotor.setControl(new VelocityVoltage(getState().getTargetRPM() / 60.0));
        
        SmartDashboard.putNumber("HDSR/currentVelocity", getCurrentVelocity());
        // SmartDashboard.putNumber("HDSR/currentAngle", getCurrentAngle().getDegrees());
    }
}
