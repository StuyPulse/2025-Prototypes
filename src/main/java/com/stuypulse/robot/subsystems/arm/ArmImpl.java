package com.stuypulse.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmImpl extends Arm {
    // Hardware
    private final TalonFX frontShoulderMotor;
    private final TalonFX backShoulderMotor;
    private final TalonFX elbowMotor;
    
    private final Pigeon2 pigeon;
    private final DutyCycleEncoder elbowEncoder;



    // Physical Constants
    private final double SHOULDER_MASS = Constants.Arm.SHOULDER_MASS; // kg
    private final double ELBOW_MASS = Constants.Arm.ELBOW_MASS;       // kg
    private final double SHOULDER_LENGTH = Constants. Arm.SHOULDER_LENGTH; // m
    private final double ELBOW_LENGTH = Constants.Arm.ELBOW_LENGTH;     // m
    private final double GRAVITY = 9.81; // m/s²

    // Conversion Factors
    private final double SHOULDER_GEAR_RATIO = Constants.Arm.SHOULDER_GEAR_RATIO;
    private final double ELBOW_GEAR_RATIO = Constants.Arm.ELBOW_GEAR_RATIO;
    private final double TORQUE_TO_VOLTAGE = 1.0 / 12.0; // Convert Nm to volts

    public ArmImpl() {
        frontShoulderMotor = new TalonFX(Ports.Arm.LEFT_SHOULDER);
        backShoulderMotor = new TalonFX(Ports.Arm.RIGHT_SHOULDER);
        elbowMotor = new TalonFX(Ports.Arm.ELBOW_MOTOR);
        pigeon = new Pigeon2(Ports.Arm.PIGEON);
        elbowEncoder = new DutyCycleEncoder(Ports.Arm.ABSOLUTE_ENCODER);
        
        Motors.Arm.SHOULDER_MOTOR_CONFIG.configure(frontShoulderMotor);
        Motors.Arm.SHOULDER_MOTOR_CONFIG.configure(backShoulderMotor);
        Motors.Arm.ELBOW_MOTOR_CONFIG.configure(elbowMotor);
        
        // Follower Setup
        backShoulderMotor.setControl(new Follower(frontShoulderMotor.getDeviceID(), false));
    }

    @Override
    public Rotation2d getShoulderAngle() {
        return Rotation2d.fromDegrees(pigeon.getPitch().getValueAsDouble());
    }

    @Override
    public Rotation2d getElbowAngle() {
        return Rotation2d.fromRotations(elbowEncoder.get());
    }

    private double calculateShoulderTorque(Rotation2d shoulder, Rotation2d elbow) {
        return (0.5 * SHOULDER_MASS * SHOULDER_LENGTH + ELBOW_MASS * SHOULDER_LENGTH) 
               * GRAVITY * Math.cos(shoulder.getRadians())
               + 0.5 * ELBOW_MASS * ELBOW_LENGTH * GRAVITY 
               * Math.cos(shoulder.plus(elbow).getRadians());
    }

    private double calculateElbowTorque(Rotation2d shoulder, Rotation2d elbow) {
        return 0.5 * ELBOW_MASS * ELBOW_LENGTH * GRAVITY 
               * Math.cos(shoulder.plus(elbow).getRadians());
    }

    @Override
    public Translation2d getEndPosition(){
        Translation2d endPosition = new Translation2d(Constants.Arm.BASE_HEIGHT, 0);
        Translation2d shoulderJoint = new Translation2d(SHOULDER_LENGTH, getShoulderAngle());
        Translation2d elbowJoint = new Translation2d(ELBOW_LENGTH, getShoulderAngle());

        endPosition.plus(shoulderJoint).plus(elbowJoint);

        return endPosition;
    }

    @Override 
    public boolean atShoulderTargetAngle(){
        return Math.abs(getState().getShoulderTargetAngle().getDegrees() - getShoulderAngle().getDegrees()) < Settings.Arm.Shoulder.TOLERANCE;
    }

    @Override 
    public boolean atElbowTargetAngle(){
        return Math.abs(getState().getElbowTargetAngle().getDegrees() - getElbowAngle().getDegrees()) < Settings.Arm.Shoulder.TOLERANCE;
    }

    @Override
    public void periodic() {
        // Calculate dynamic gravity compensation
        double shoulderFF = calculateShoulderTorque(getShoulderAngle(), getElbowAngle()) 
                          / (SHOULDER_GEAR_RATIO * 12.0);
        double elbowFF = calculateElbowTorque(getShoulderAngle(), getElbowAngle()) 
                       / (ELBOW_GEAR_RATIO * 12.0);
        
        frontShoulderMotor.setControl(new MotionMagicVoltage(
                getState().getShoulderTargetAngle().getRotations())
                .withFeedForward(shoulderFF)
        );
        
        elbowMotor.setControl(new MotionMagicVoltage(
                getState().getElbowTargetAngle().getRotations())
                .withFeedForward(elbowFF)
        );


        // Logging
        SmartDashboard.putNumber("Arm/Shoulder Angle", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Elbow Angle", getElbowAngle().getDegrees());

        SmartDashboard.putNumber("Arm/End Length?", getEndPosition().getX());
        SmartDashboard.putNumber("Arm/End Height", getEndPosition().getY());

        SmartDashboard.putNumber("Arm/Shoulder Torque", calculateShoulderTorque(getShoulderAngle(), getElbowAngle()));
        SmartDashboard.putNumber("Arm/Elbow Torque", calculateElbowTorque(getShoulderAngle(), getElbowAngle()));

        SmartDashboard.putBoolean("Arm/At Shoulder Target Angle", atShoulderTargetAngle());
        SmartDashboard.putBoolean("Arm/At Elbow Target Angle", atElbowTargetAngle());
    }
}