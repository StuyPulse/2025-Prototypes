package com.stuypulse.robot.subsystems.spindexer.spinner;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

public class SpinnerImpl extends Spinner {
    private SparkMax motor;

    public SpinnerImpl() {
        motor = new SparkMax(Ports.Spindexer.SPINNER_MOTOR, MotorType.kBrushless);
        motor.configure(Motors.Spindexer.SPINDEXER_SPINNER, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        motor.set(getState().getSpeed());
    }
}
