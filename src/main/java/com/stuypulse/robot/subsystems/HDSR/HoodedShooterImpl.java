package com.stuypulse.robot.subsystems.HDSR;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.booleans.BStream;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Motors.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class HoodedShooterImpl extends HoodedShooter {
    private final TalonFX hoodMotor;
    private final TalonFX rollerMotor;
    private final TalonFX shooterMotor;

    private final AnglePIDController hoodController;
    private final PIDController shooterController;

    private final DutyCycle rollerCycle;

    public HoodedShooterImpl() {
        super();


        hoodMotor = new TalonFX(Ports.HDSR.HOOD_MOTOR);
        rollerMotor = new TalonFX(Ports.HDSR.ROLLER_MOTOR);
        shooterMotor = new TalonFX(Ports.HDSR.SHOOTER_MOTOR);

        
        TalonFXConfig shooterMotorConfig = Motors.SHOOTER_MOTOR_CONFIG;
        shooterMotorConfig.configure(shooterMotor);

        TalonFXConfig rollerMotorConfig = Motors.ROLLER_MOTOR_CONFIG;
        rollerMotorConfig.configure(rollerMotor);

        TalonFXConfig hoodMotorConfig = Motors.HOOD_MOTOR_CONFIG;
        hoodMotorConfig.configure(hoodMotor);

        hoodController = new AnglePIDController(Gains.HDSR.kP, Gains.HDSR.kI, Gains.HDSR.kD);

    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(hoodMotor.getPosition().getValueAsDouble());
    }

    public double getCurrentVelocity() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

   
   
    @Override
   
    public void periodic() {
        super.periodic();

        hoodController.update(Angle.fromDegrees(getTargetAngle().getDegrees()), Angle.fromDegrees(getCurrentAngle().getDegrees()));
        hoodMotor.setVoltage(hoodController.getOutput());

        shooterController.update(getTargetVelocity(), getCurrentVelocity());
        shooterMotor.setVoltage(shooterController.getOutput());

        rollerMotor.set(Settings.HDSR.ROLLER_SPEED);

        

        SmartDashboard.putNumber("HDSR/currentVelocity", getCurrentVelocity());
        SmartDashboard.putNumber("HDSR/currentAngle", getCurrentAngle().getDegrees());
    }
}
