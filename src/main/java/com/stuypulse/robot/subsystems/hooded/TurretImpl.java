package com.stuypulse.robot.subsystems.hooded;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.stuypulse.robot.constants.Ports;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class TurretImpl extends Turret {

    private TalonFX motor;
    private CANcoder encoder;

    public TurretImpl() {
        motor = new TalonFX(0);
    }

    @Override
    public double getAngle() {
        return motor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {

    }
}
