package com.stuypulse.robot.subsystems.spindexerKraken;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpindexerKrakenImpl extends SpindexerKraken{
    private static TalonFX spindexerKrakenMotor;

    public SpindexerKrakenImpl() {
        spindexerKrakenMotor = new TalonFX(Ports.Spindexer.SPINDEXER_KRAKEN_MOTOR);

        Motors.Spindexer.SPINDEXER_KRAKEN_MOTOR_CONFIG.configure(spindexerKrakenMotor);      
    }

    public static double getCurrentDutyCycle() {
        return spindexerKrakenMotor.getDutyCycle().getValueAsDouble();
    }

    public static Supplier<Boolean> atDutyCycle() {
        return () -> getCurrentDutyCycle() >= 1.0; //VALUE FOR SIM: 0.08002
    }

    @Override
    public void periodic() {
        spindexerKrakenMotor.setControl(new DutyCycleOut(getKrakenState().getSpindexerSpeed()));
        
        SmartDashboard.putNumber("Spindexer/ Kraken Target Speed (State Fraction)", getKrakenState().getSpindexerSpeed());
        SmartDashboard.putNumber("Spindexer/ Kraken Speed (Motor Duty Cycle)", getCurrentDutyCycle());
        SmartDashboard.putBoolean("Spindexer/ Kraken at target dutyCycle?", getCurrentDutyCycle() >= 0.08);

        SmartDashboard.putNumber("Spindexer/ Kraken Voltage", spindexerKrakenMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Spindexer/ Kraken Current", spindexerKrakenMotor.getSupplyCurrent().getValueAsDouble());
    }
}   
