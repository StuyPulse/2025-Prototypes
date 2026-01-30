package com.stuypulse.robot.subsystems.spindexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpindexerImpl extends Spindexer{
    private TalonFX spindexerMotor;

    public SpindexerImpl() {
        super();

        spindexerMotor = new TalonFX(Ports.Spindexer.SPINDEXER_MOTOR);

        Motors.Spindexer.SPINDEXER_MOTOR_CONFIG.configure(spindexerMotor);      
    }

    @Override
    public void periodic() {
        spindexerMotor.setControl(new DutyCycleOut(getState().getSpindexerSpeed()));

        SmartDashboard.putString("SPINDEXER/ STATE", getState().toString());
    }
}   
