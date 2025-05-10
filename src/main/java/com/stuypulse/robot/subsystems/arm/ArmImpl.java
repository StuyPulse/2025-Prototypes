package com.stuypulse.robot.subsystems.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    private final PIDController shoulderPID = new PIDController(0.5, 0.0, 0.1);
    private final PIDController elbowPID = new PIDController(0.6, 0.0, 0.15);
    private final ArmFeedforward shoulderFF = new ArmFeedforward(0.01, 0.15, 0.02, 0.05);
    private final ArmFeedforward elbowFF = new ArmFeedforward(0.01, 0.08, 0.01, 0.03);

    // Arm geometry (meters)
    private final double SHOULDER_LENGTH = 0.5;
    private final double ELBOW_LENGTH = 0.4;
    private final double BASE_HEIGHT = 0.2;

    // Dynamic coupling
    private final double SHOULDER_TO_ELBOW_COUPLING = 0.3;
    private final double ELBOW_TO_SHOULDER_COUPLING = 0.2;

    // Targets
    private double targetShoulderDeg = 0;
    private double targetElbowDeg = 0;

    // Elbow encoder calibration
    private final double ELBOW_OFFSET_DEG = 0.0; // Calibrate this
    private double prevElbowPos = 0;
    private long prevTime = System.nanoTime();

    public ArmImpl() {
        frontShoulderMotor = new TalonFX(1);
        backShoulderMotor = new TalonFX(2);
        elbowMotor = new TalonFX(3);
        
        shoulderEncoder = new Pigeon2(4);
        elbowEncoder = new DutyCycleEncoder(0);
        
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
    public void periodic() {
        // Get current states
        double shoulderPos = getShoulderAngleDegrees();
        double elbowPos = getElbowAngleDegrees();
        double shoulderVel = getShoulderVelocityDegs();
        double elbowVel = getElbowVelocityDegs();

        // Calculate coupled feedforward
        double shoulderGravity = calculateShoulderGravity(shoulderPos, elbowPos);
        double elbowGravity = calculateElbowGravity(shoulderPos, elbowPos);
        
        // Cross-joint compensation
        double shoulderComp = ELBOW_TO_SHOULDER_COUPLING * elbowVel;
        double elbowComp = SHOULDER_TO_ELBOW_COUPLING * shoulderVel;

        // Control signals
        double shoulderVolts = shoulderPID.calculate(shoulderPos, targetShoulderDeg)
                           + shoulderFF.calculate(shoulderVel + shoulderComp, shoulderGravity);
        
        double elbowVolts = elbowPID.calculate(elbowPos, targetElbowDeg)
                         + elbowFF.calculate(elbowVel + elbowComp, elbowGravity);

        setVoltages(shoulderVolts, elbowVolts);
        
        // Logging
        SmartDashboard.putNumber("Arm/Shoulder Angle", shoulderPos);
        SmartDashboard.putNumber("Arm/Elbow Angle", elbowPos);
        SmartDashboard.putNumber("Arm/End Height", getEndHeight());
    }

    private double calculateShoulderGravity(double shoulderDeg, double elbowDeg) {
        double shoulderRad = Math.toRadians(shoulderDeg);
        double elbowRad = Math.toRadians(elbowDeg);
        return 0.15 * Math.sin(shoulderRad) + 0.05 * Math.sin(shoulderRad + elbowRad);
    }

    private double calculateElbowGravity(double shoulderDeg, double elbowDeg) {
        double combinedRad = Math.toRadians(shoulderDeg + elbowDeg);
        return 0.08 * Math.sin(combinedRad);
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
    public double getShoulderVelocityDegs() {
        return shoulderEncoder.getAngularVelocityZWorld().getValueAsDouble();
    }

    @Override
    public double getElbowVelocityDegs() {
        long currentTime = System.nanoTime();
        double dt = (currentTime - prevTime) * 1e-9; // Convert to seconds
        double currentPos = getElbowAngleDegrees();
        double velocity = (currentPos - prevElbowPos) / dt;
        
        prevElbowPos = currentPos;
        prevTime = currentTime;
        
        return velocity;
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
            SHOULDER_LENGTH * Math.cos(shoulderRad) + ELBOW_LENGTH * Math.cos(shoulderRad + elbowRad),
            SHOULDER_LENGTH * Math.sin(shoulderRad) + ELBOW_LENGTH * Math.sin(shoulderRad + elbowRad)
        );
    }

    @Override
    public double getEndHeight() {
        return BASE_HEIGHT + getEndPosition().getY();
    }

    @Override
    public void setVoltages(double shoulderVolts, double elbowVolts) {
        frontShoulderMotor.setVoltage(shoulderVolts);
        elbowMotor.setVoltage(elbowVolts);
    }

    @Override
    public void stop() {
        setVoltages(0, 0);
    }

    @Override
    public void resetEncoders() {
        shoulderEncoder.setYaw(0);
        //How to reset DutyCycleEncoder??!?!?
    }
}