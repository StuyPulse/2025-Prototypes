package com.stuypulse.robot.subsystems.hooded;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.stuypulse.robot.constants.Ports.HoodedShooter;
import com.stuypulse.robot.constants.Settings;

public class HoodImpl extends Hood {
    private SparkMax motor;
    private CANcoder encoder;

    public HoodImpl() {
        motor = new SparkMax(HoodedShooter.MOTOR, MotorType.kBrushless);
        encoder = new CANcoder(HoodedShooter.ENCODER, HoodedShooter.CANBUS);
    }

    public double getAngle() {
        return encoder.getAbsolutePosition().getValueAsDouble();
    }

    public boolean atTargetAngle() {
        return Math.abs(getAngle() - getTargetAngle()) < Settings.HoodedShooter.tolerance;
    }

}
