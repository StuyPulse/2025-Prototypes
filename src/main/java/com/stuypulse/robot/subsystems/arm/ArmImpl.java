package com.stuypulse.robot.subsystems.arm;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
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
import edu.wpi.first.math.geometry.Transform2d;
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

    private final PositionVoltage shoulderPositionReq = new PositionVoltage(0).withSlot(0);
    private final PositionVoltage elbowPositionReq = new PositionVoltage(0).withSlot(1);

    public ArmImpl() {
        frontShoulderMotor = new TalonFX(Ports.Arm.LEFT_SHOULDER);
        backShoulderMotor = new TalonFX(Ports.Arm.RIGHT_SHOULDER);
        elbowMotor = new TalonFX(Ports.Arm.ELBOW_MOTOR);
        pigeon = new Pigeon2(Ports.Arm.PIGEON);
        elbowEncoder = new DutyCycleEncoder(Ports.Arm.ABSOLUTE_ENCODER);
        
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration masterShoulderConfig = new TalonFXConfiguration();
        TalonFXConfiguration followerShoulderConfig = new TalonFXConfiguration();
        TalonFXConfiguration elbowConfig = new TalonFXConfiguration();
        
        // Shoulder Configuration
        Slot0Configs shoulderSlot0 = masterShoulderConfig.Slot0;
        shoulderSlot0.withKP(Settings.Arm.Shoulder.PID.kP)
                    .withKI(Settings.Arm.Shoulder.PID.kI)
                    .withKD(Settings.Arm.Shoulder.PID.kD)
                    .withKS(Settings.Arm.Shoulder.FF.kS)  
                    .withKV(Settings.Arm.Shoulder.FF.kV)  
                    .withKA(Settings.Arm.Shoulder.FF.kA); 

        // Elbow Configuration
        Slot1Configs elbowSlot1 = elbowConfig.Slot1;
        elbowSlot1.withKP(Settings.Arm.Elbow.PID.kP)
                    .withKI(Settings.Arm.Elbow.PID.kI)
                    .withKD(Settings.Arm.Elbow.PID.kD)
                    .withKS(Settings.Arm.Elbow.FF.kS)
                    .withKV(Settings.Arm.Elbow.FF.kV)
                    .withKA(Settings.Arm.Elbow.FF.kA);
        
        // Master Motor Configuration
        masterShoulderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        masterShoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        frontShoulderMotor.getConfigurator().apply(masterShoulderConfig);

        // Follower Motor Configuration
        followerShoulderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        followerShoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        backShoulderMotor.getConfigurator().apply(followerShoulderConfig);

        backShoulderMotor.setControl(new Follower(frontShoulderMotor.getDeviceID(), false));

        elbowConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elbowConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elbowMotor.getConfigurator().apply(elbowConfig);
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
        return GRAVITY * (SHOULDER_MASS * (SHOULDER_LENGTH * 0.5) * Math.cos(shoulder.getDegrees()) + ELBOW_MASS * 
                (SHOULDER_LENGTH * Math.cos(shoulder.getDegrees())  + (ELBOW_LENGTH * 0.5) * Math.cos(shoulder.getDegrees() * elbow.getDegrees())));
    }

    private double calculateElbowTorque(Rotation2d shoulder, Rotation2d elbow) {
        return GRAVITY * ELBOW_MASS * (ELBOW_LENGTH * 0.5) * Math.cos(shoulder.getDegrees() + elbow.getDegrees());
    }

    @Override
    public void setTargetAngles(Rotation2d shoulder, Rotation2d elbow) {
        Rotation2d currentShoulder = getShoulderAngle();
        Rotation2d currentElbow = getElbowAngle();
        
        // Convert torque to volts + compensate for gear ratios
        double shoulderVolts = calculateShoulderTorque(currentShoulder, currentElbow) 
                            / (SHOULDER_GEAR_RATIO * 12.0);
        double elbowVolts = calculateElbowTorque(currentShoulder, currentElbow) 
                         / (ELBOW_GEAR_RATIO * 12.0);
        
        frontShoulderMotor.setControl(
            shoulderPositionReq
                .withPosition(shoulder.getRotations())
                .withFeedForward(shoulderVolts)
        );
        
        elbowMotor.setControl(
            elbowPositionReq
                .withPosition(elbow.getRotations())
                .withFeedForward(elbowVolts)
        );
    }

    @Override
    public Translation2d getEndPosition() {
        Rotation2d shoulder = getShoulderAngle();
        Rotation2d elbow = getElbowAngle();

        Transform2d startPoint = new Transform2d(0.0, Constants.Arm.BASE_HEIGHT, new Rotation2d(0.0));
        Translation2d endPoint = startPoint.plus(new Transform2d(Constants.Arm.SHOULDER_LENGTH, 0.0, shoulder))
                                .plus(new Transform2d(Constants.Arm.ELBOW_LENGTH, 0.0, elbow.minus(new Rotation2d(180.0 - shoulder.getDegrees()))))
                                .getTranslation();
        return endPoint;
    }

    @Override
    public void periodic() {
        // Logging
        SmartDashboard.putNumber("Arm/Shoulder Angle", getShoulderAngle().getDegrees());
        SmartDashboard.putNumber("Arm/Elbow Angle", getElbowAngle().getDegrees());
        SmartDashboard.putNumber("Arm/End Height", getEndPosition().getY());
        
        Rotation2d shoulder = getShoulderAngle();
        Rotation2d elbow = getElbowAngle();
        SmartDashboard.putNumber("Arm/Shoulder Torque", calculateShoulderTorque(shoulder, elbow));
        SmartDashboard.putNumber("Arm/Elbow Torque", calculateElbowTorque(shoulder, elbow));
    }
}