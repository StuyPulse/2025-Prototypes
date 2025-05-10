package com.stuypulse.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmImpl extends Arm {
    // Hardware
    private final Pigeon2 pigeon;
    private final CANcoder elbowEncoder;
    private final TalonFX shoulderMotor;
    private final TalonFX elbowMotor;

    // Degree-based controllers
    private final PIDController shoulderPID = new PIDController(0.5, 0.0, 0.1);
    private final PIDController elbowPID = new PIDController(0.6, 0.0, 0.15);
    private final ArmFeedforward shoulderFF = new ArmFeedforward(0.0, 0.15, 0.02);
    private final ArmFeedforward elbowFF = new ArmFeedforward(0.0, 0.08, 0.01);

    // Arm geometry (meters)
    private final double SHOULDER_LENGTH = 0.5;
    private final double ELBOW_LENGTH = 0.4;
    private final double BASE_HEIGHT = 0.2;

    // Targets in degrees
    private double targetShoulderDeg = 0;
    private double targetElbowDeg = 0;

    public ArmImpl() {
        pigeon = new Pigeon2(0);
        elbowEncoder = new CANcoder(1);
        shoulderMotor = new TalonFX(2);
        elbowMotor = new TalonFX(3);
        
        configureCANcoder();
    }

    private void configureCANcoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        
        // Current (2024) way to configure sensor range
        config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        
        // No explicit range setting needed - default is 0-1 (0-360° when multiplied)
        var status = elbowEncoder.getConfigurator().apply(config);
        
        if (!status.isOK()) {
            DriverStation.reportWarning("CANcoder config failed!", false);
        }
    }

    @Override
    public void periodic() {
        // Degree-based control
        double shoulderVolts = shoulderPID.calculate(getShoulderAngleDegrees(), targetShoulderDeg)
                           + shoulderFF.calculate(getShoulderAngleDegrees(), getShoulderVelocityDegS());
        
        double elbowVolts = elbowPID.calculate(getElbowAngleDegrees(), targetElbowDeg)
                         + elbowFF.calculate(getElbowAngleDegrees(), getElbowVelocityDegS());
        
        setVoltages(shoulderVolts, elbowVolts);
        
        // Logging
        SmartDashboard.putNumber("Arm/Shoulder Angle (deg)", getShoulderAngleDegrees());
        SmartDashboard.putNumber("Arm/Elbow Angle (deg)", getElbowAngleDegrees());
        SmartDashboard.putNumber("Arm/End Height (m)", getEndHeight());
    }

    // Degree-based measurements
    public double getShoulderAngleDegrees() {
        return pigeon.getPitch().getValueAsDouble();
    }

    public double getElbowAngleDegrees() {
        // Convert 0-1 to 0-360°
        return elbowEncoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }

    public double getShoulderVelocityDegS() {
        return pigeon.getAngularVelocityZWorld().getValueAsDouble();
    }

    public double getElbowVelocityDegS() {
        // Convert RPM to deg/s (360°/60s = 6)
        return elbowEncoder.getVelocity().getValueAsDouble() * 6.0;
    }

    // Degree-based control
    public void setTargetAngles(double shoulderDeg, double elbowDeg) {
        targetShoulderDeg = shoulderDeg;
        targetElbowDeg = elbowDeg;
    }

    // Kinematics (internal conversion to radians)
    public Translation2d getEndPosition() {
        double shoulderRad = Math.toRadians(getShoulderAngleDegrees());
        double elbowRad = Math.toRadians(getElbowAngleDegrees());
        
        double x = SHOULDER_LENGTH * Math.cos(shoulderRad) 
                 + ELBOW_LENGTH * Math.cos(shoulderRad + elbowRad);
        double y = SHOULDER_LENGTH * Math.sin(shoulderRad) 
                 + ELBOW_LENGTH * Math.sin(shoulderRad + elbowRad);
        
        return new Translation2d(x, y);
    }

    public double getEndHeight() {
        return BASE_HEIGHT + getEndPosition().getY();
    }

    @Override
    public void setVoltages(double shoulderVolts, double elbowVolts) {
        shoulderMotor.setVoltage(shoulderVolts);
        elbowMotor.setVoltage(elbowVolts);
    }

    @Override
    public void stop() {
        setVoltages(0, 0);
    }

    @Override
    public void resetEncoders() {
        elbowEncoder.setPosition(0);
        pigeon.setYaw(0);
    }
}