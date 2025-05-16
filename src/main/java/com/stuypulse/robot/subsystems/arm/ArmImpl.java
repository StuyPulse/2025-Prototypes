package com.stuypulse.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmImpl extends Arm {
    // Hardware
    private final TalonFX frontShoulderMotor;
    private final TalonFX backShoulderMotor;
    private final TalonFX elbowMotor;
    
    private final Pigeon2 shoulderEncoder;
    private final DutyCycleEncoder elbowEncoder;

    /*
     CHANGE VALUES
     */

    // Control
    private final PIDController shoulderPID = new PIDController(
        Settings.Arm.Shoulder.PID.kP,
        Settings.Arm.Shoulder.PID.kI,
        Settings.Arm.Shoulder.PID.kD
        );
    private final PIDController elbowPID = new PIDController(
        Settings.Arm.Elbow.PID.kP,
        Settings.Arm.Elbow.PID.kI,
        Settings.Arm.Elbow.PID.kD
        );
    private final ArmFeedforward shoulderFF = new ArmFeedforward(
        Settings.Arm.Shoulder.FF.kS,
        Settings.Arm.Shoulder.FF.kG,
        Settings.Arm.Shoulder.FF.kV,
        Settings.Arm.Shoulder.FF.kA
    );
    private final ArmFeedforward elbowFF = new ArmFeedforward(
        Settings.Arm.Elbow.FF.kS,
        Settings.Arm.Elbow.FF.kG,
        Settings.Arm.Elbow.FF.kV,
        Settings.Arm.Elbow.FF.kA
    );

    // Targets
    private double targetShoulderDeg = 0;
    private double targetElbowDeg = 0;

    // Elbow encoder calibration
    private final double ELBOW_OFFSET_DEG = 0.0; // Calibrate this
    private double prevElbowPos = 0;
    private long prevTime = System.nanoTime();

    public ArmImpl() {
        this.state = ArmState.RESTING;
        frontShoulderMotor = new TalonFX(Ports.Arm.LEFT_SHOULDER);
        backShoulderMotor = new TalonFX(Ports.Arm.RIGHT_SHOULDER);
        elbowMotor = new TalonFX(Ports.Arm.ELBOW_MOTOR);
        shoulderEncoder = new Pigeon2(Ports.Arm.PIGEON);
        elbowEncoder = new DutyCycleEncoder(Ports.Arm.ABSOLUTE_ENCODER);
        
        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Shoulder motors
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        frontShoulderMotor.getConfigurator().apply(config);
        backShoulderMotor.getConfigurator().apply(config);
        
        // Elbow motor
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        elbowMotor.getConfigurator().apply(config);
        
        // Follower setup
        backShoulderMotor.setControl(new Follower(frontShoulderMotor.getDeviceID(), false));
    }


    @Override
    public double getShoulderAngleDegrees() {
        return shoulderEncoder.getPitch().getValueAsDouble();
    }

    @Override
    public double getElbowAngleDegrees() {
        return (elbowEncoder.get() * 360.0) - ELBOW_OFFSET_DEG;
    }

    @Override
    public void setTargetAngles(double shoulderDeg, double elbowDeg) {
        targetShoulderDeg = shoulderDeg;
        targetElbowDeg = elbowDeg;
    }

    @Override
    public Translation2d getEndPosition() {
        double shoulderRad = Math.toRadians(getShoulderAngleDegrees());
        double elbowRad = Math.toRadians(getElbowAngleDegrees());
        
        return new Translation2d(
            Constants.Arm.SHOULDER_LENGTH * Math.cos(shoulderRad) + Constants.Arm.ELBOW_LENGTH * Math.cos(shoulderRad + elbowRad),
            Constants.Arm.SHOULDER_LENGTH * Math.sin(shoulderRad) + Constants.Arm.ELBOW_LENGTH * Math.sin(shoulderRad + elbowRad)
        );
    }

    @Override
    public void periodic() {
        double shoulderPos = getShoulderAngleDegrees();
        double elbowPos = getElbowAngleDegrees();
        
        // Logging
        SmartDashboard.putNumber("Arm/Shoulder Angle", shoulderPos);
        SmartDashboard.putNumber("Arm/Elbow Angle", elbowPos);
        SmartDashboard.putNumber("Arm/End Height", getEndPosition().getY());
    }


}