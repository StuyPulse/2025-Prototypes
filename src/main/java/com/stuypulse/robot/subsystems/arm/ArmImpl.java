package com.stuypulse.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Constants;
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

    // Control Requests
    private final PositionVoltage shoulderPositionReq = new PositionVoltage(0)
        .withSlot(0);
    private final PositionVoltage elbowPositionReq = new PositionVoltage(0)
        .withSlot(1);

    // Targets
    private Rotation2d targetShoulderAngle = Rotation2d.fromDegrees(0);
    private Rotation2d targetElbowAngle = Rotation2d.fromDegrees(0);

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
        
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Shoulder PID Config
        config.Slot0.kP = Settings.Arm.Shoulder.PID.kP;
        config.Slot0.kI = Settings.Arm.Shoulder.PID.kI;
        config.Slot0.kD = Settings.Arm.Shoulder.PID.kD;

        // Elbow PID Config
        config.Slot1.kP = Settings.Arm.Elbow.PID.kP;
        config.Slot1.kI = Settings.Arm.Elbow.PID.kI;
        config.Slot1.kD = Settings.Arm.Elbow.PID.kD;

        // Motor Inversion
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        frontShoulderMotor.getConfigurator().apply(config);
        backShoulderMotor.getConfigurator().apply(config);
        
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elbowMotor.getConfigurator().apply(config);
        
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
    public void setTargetAngles(Rotation2d shoulder, Rotation2d elbow) {
        targetShoulderAngle = shoulder;
        targetElbowAngle = elbow;
        
        Rotation2d currentShoulder = getShoulderAngle();
        Rotation2d currentElbow = getElbowAngle();
        
        // Calculate dynamic gravity compensation
        double shoulderFF = calculateShoulderTorque(currentShoulder, currentElbow) 
                          / (SHOULDER_GEAR_RATIO * 12.0);
        double elbowFF = calculateElbowTorque(currentShoulder, currentElbow) 
                       / (ELBOW_GEAR_RATIO * 12.0);
        
        frontShoulderMotor.setControl(
            shoulderPositionReq
                .withPosition(shoulder.getRotations())
                .withFeedForward(shoulderFF)
        );
        
        elbowMotor.setControl(
            elbowPositionReq
                .withPosition(elbow.getRotations())
                .withFeedForward(elbowFF)
        );
    }

    @Override
    public double getEndHeight() {
        Rotation2d shoulder = getShoulderAngle();
        Rotation2d elbow = getElbowAngle();
        
        return SHOULDER_LENGTH * Math.sin(shoulder.getRadians()) 
             + ELBOW_LENGTH * Math.sin(shoulder.plus(elbow).getRadians());
    } //FIX TS LATER

    @Override
    public void periodic() {
        // Logging
        SmartDashboard.putNumber("Arm/Shoulder Angle", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Elbow Angle", getElbowAngle().getDegrees());
        SmartDashboard.putNumber("Arm/End Height", getEndHeight());
        
        
        Rotation2d shoulder = getShoulderAngle();
        Rotation2d elbow = getElbowAngle();
        SmartDashboard.putNumber("Arm/Shoulder Torque", calculateShoulderTorque(shoulder, elbow));
        SmartDashboard.putNumber("Arm/Elbow Torque", calculateElbowTorque(shoulder, elbow));
    }
}