package com.stuypulse.robot.subsystems.spindexer.roller;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

public class RollerImpl extends Roller{
    private TalonFX spindexerMotor;

    public RollerImpl() {
        super();

        spindexerMotor = new TalonFX(Ports.Spindexer.ROLLER_MOTOR);

        Motors.Spindexer.SPINDEXER_ROLLER.configure(spindexerMotor);      
    }

    @Override
    public void periodic() {
        spindexerMotor.setControl(new DutyCycleOut(getState().getSpeed()));
    }
}   
