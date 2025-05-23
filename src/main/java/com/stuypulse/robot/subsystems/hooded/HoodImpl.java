package com.stuypulse.robot.subsystems.hooded;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Ports.HoodedShooter;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.angle.AngleController;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;

import edu.wpi.first.math.geometry.Rotation2d;

public class HoodImpl extends Hood {
    private SparkMax motor;
    private CANcoder encoder;

    private AngleController controller;

    private SparkBaseConfig motorConfig;
    private CANcoderConfiguration encoderConfig;

    public HoodImpl() {
        super();
        motor = new SparkMax(HoodedShooter.MOTOR, MotorType.kBrushless);
        encoder = new CANcoder(HoodedShooter.ENCODER, HoodedShooter.CANBUS);
        

        motorConfig = new SparkMaxConfig();
        encoderConfig = new CANcoderConfiguration();

        // configs
        motorConfig.closedLoopRampRate(2.0)
                    .smartCurrentLimit(Constants.HoodedShooter.SMART_CURRENT_LIMIT)
                    .secondaryCurrentLimit(0)
                    .inverted(false);
        motor.configure(motorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
        
        encoderConfig.MagnetSensor.withMagnetOffset(Constants.HoodedShooter.Encoders.OFFSET)
                                    .withAbsoluteSensorDiscontinuityPoint(1)
                                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        encoder.getConfigurator().apply(encoderConfig);
        
        controller = new AnglePIDController(Constants.HoodedShooter.kP, Constants.HoodedShooter.kI, Constants.HoodedShooter.kD)
                    .setSetpointFilter(new AMotionProfile(Constants.HoodedShooter.DEFAULT_MAX_ANGULAR_VELOCITY, Constants.HoodedShooter.DEFAULT_MAX_ANGULAR_ACCELERATION));
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public boolean atTargetAngle() {
        return Math.abs(getAngle().getDegrees() - getState().getTargetAngle().getDegrees()) < Settings.HoodedShooter.TOLERANCE;
    }
    
    @Override
    public void periodic() {
        super.periodic();
        controller.update(Angle.fromRotations(getState().getTargetAngle().getRotations()), Angle.fromRotations(getAngle().getRotations()));
        double voltage = controller.getOutput();
        
        if (Math.abs(voltage) < 0.05) {
            motor.setVoltage(0);
        } else {
            motor.setVoltage(voltage);
        }
    }
}
