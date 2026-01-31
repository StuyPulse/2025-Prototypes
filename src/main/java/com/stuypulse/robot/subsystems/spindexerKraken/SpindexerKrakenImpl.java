package com.stuypulse.robot.subsystems.spindexerKraken;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpindexerKrakenImpl extends SpindexerKraken{
    private TalonFX spindexerKrakenMotor;

    public SpindexerKrakenImpl() {
        spindexerKrakenMotor = new TalonFX(Ports.Spindexer.SPINDEXER_KRAKEN_MOTOR);

        Motors.Spindexer.SPINDEXER_KRAKEN_MOTOR_CONFIG.configure(spindexerKrakenMotor);      
    }

    @Override
    public void periodic() {
        spindexerKrakenMotor.setControl(new DutyCycleOut(getKrakenState().getSpindexerSpeed()));
        
        SmartDashboard.putNumber("Spindexer/ Kraken Speed", getKrakenState().getSpindexerSpeed());

    }
}   
