package com.stuypulse.robot.subsystems.hooded;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.stuypulse.robot.constants.Ports;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class TurretImpl extends Turret {

    private SparkMax motor;
    private AbsoluteEncoder encoder;



    public TurretImpl() {
        this.motor = new SparkMax(Ports.Turret.MOTOR, MotorType.kBrushless);
        encoder = motor.getAbsoluteEncoder();
    }

    @Override
    public double getAngle() {
        return encoder.getPosition();
    }

    @Override
    public void periodic() {

    }
}
