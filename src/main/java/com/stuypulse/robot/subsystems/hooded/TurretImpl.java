package com.stuypulse.robot.subsystems.hooded;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretImpl extends Turret {

    private TalonFX motor;
    private CANcoder encoder;

    CANcoderConfiguration encoderConfig;

    public TurretImpl() {
        motor = new TalonFX(Ports.Turret.MOTOR);
        encoder = new CANcoder(Ports.Turret.ENCODER, Ports.Turret.CANBUS);

        // TalonFX config
        Motors.Turret.MOTOR_CONFIG.configure(motor);
        motor.setPosition(0);

        encoderConfig.MagnetSensor.withMagnetOffset(Constants.Turret.Encoders.OFFSET)
                                    .withAbsoluteSensorDiscontinuityPoint(1)
                                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);
        encoder.getConfigurator().apply(encoderConfig);
        
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
    }

    @Override
    public boolean atTargetAngle() {
        return Math.abs(getAngle().getDegrees() - getTargetAngle().getDegrees()) < Settings.Turret.TOLERANCE;
    }

    @Override
    public void periodic() {
        super.periodic();
        motor.setControl(new MotionMagicVoltage(getTargetAngle().getDegrees()));
    }
}
