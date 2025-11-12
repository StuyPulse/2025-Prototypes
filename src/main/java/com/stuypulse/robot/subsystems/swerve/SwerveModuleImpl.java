package com.stuypulse.robot.subsystems.swerve;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
//import com.stuypulse.stuylib.control.feedback.PIDController;
import edu.wpi.first.math.controller.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Angle;

import edu.wpi.first.hal.CANAPIJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleImpl extends SwerveModule {

    private final Rotation2d angleOffset;
    private SwerveModuleState targetState;

    private final SparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    // private final Controller driveController;

    private final SparkMax pivotMotor;
    private final CANcoder pivotEncoder;

    // private final AngleController pivotController;
    private final PIDController pivotController;


    public SwerveModuleImpl(String name, Translation2d location, Rotation2d angleOffset, int driveMotorID, int pivotMotorID, int pivotEncoderID) {
        super(name, location);

        this.angleOffset = angleOffset;
        targetState = new SwerveModuleState();

        pivotMotor = new SparkMax(pivotMotorID, MotorType.kBrushless);
        pivotMotor.configure(Motors.Swerve.Turn.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pivotEncoder = new CANcoder(pivotEncoderID);

        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveMotor.configure(Motors.Swerve.Turn.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // driveController = 
            // new PIDController(Drive.kP, Drive.kI, Drive.kD)
                // .add(new MotorFeedforward(Drive.kS, Drive.kV, Drive.kA).velocity());
        pivotController = new PIDController(Turn.kP, Turn.kI, Turn.kD);
        pivotController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public double getVelocity() {
        return driveEncoder.getVelocity();
    }

    @Override
    public Rotation2d getAngle() {
        // double raw = pivotEncoder.getAbsolutePosition().getValueAsDouble() - angleOffset.getRotations() + 1;
        // return Rotation2d.fromRotations(raw % 1f);
        return Rotation2d.fromRotations((pivotEncoder.getAbsolutePosition().getValueAsDouble()));
    }

    public double getDistance(){
        return driveEncoder.getPosition();
    }

    @Override
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }


    public boolean atTargetAngle() {
        SwerveModuleState targetState = getTargetState();
        return 0.5 > Math.abs(targetState.angle.getDegrees() - getAngle().getDegrees());
    }

    @Override
    public void periodic() {
        super.periodic();

        double voltage = pivotController.calculate(getAngle().getRadians(), getTargetState().angle.getRadians());


        if (Math.abs(getTargetState().speedMetersPerSecond) < Settings.Swerve.MODULE_VELOCITY_DEADBAND || atTargetAngle()) {
            driveMotor.setVoltage(0);
            pivotMotor.setVoltage(0);
        } else {
            driveMotor.setVoltage(0);
            pivotMotor.setVoltage(voltage); 
        }


        SmartDashboard.putBoolean("Swerve/Modules" + getName() + "/At Target Angle", atTargetAngle());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Drive Target Speed", getTargetState().speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Drive Current", driveMotor.getOutputCurrent());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Drive Voltage", driveMotor.getBusVoltage());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Turn Voltage", voltage);
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Turn Current", pivotMotor.getOutputCurrent());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Angle Error", pivotController.getError());
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Raw Encoder Angle", pivotEncoder.getAbsolutePosition().getValueAsDouble());
    }
}