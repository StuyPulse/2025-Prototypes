package com.stuypulse.robot.subsystems.spindexerKraken;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpindexerKrakenImpl extends SpindexerKraken{
    private static TalonFX spindexerKrakenMotor;

    public SpindexerKrakenImpl() {
        spindexerKrakenMotor = new TalonFX(Ports.Spindexer.SPINDEXER_KRAKEN_MOTOR);

        Motors.Spindexer.SPINDEXER_KRAKEN_MOTOR_CONFIG.configure(spindexerKrakenMotor);      
    }

    public static Supplier<Double> getTargetSpeed() {
        return getKrakenState().getSpindexerSpeed(); 
    }
    public static double getCurrentDutyCycle() { 

        return spindexerKrakenMotor.getDutyCycle().getValueAsDouble();
    }

    public static Supplier<Boolean> atDutyCycle() { 

        return () -> getCurrentDutyCycle() >= 0.08002; //TODO: ENSURE VALUE IS CORRECT
        // VALUE FOR SIM: 0.08002
        // VALUE FOR ACTUAL TESTING: 1.0
    }

    public static Supplier<Double> getSettingValue() {
        return () -> Settings.Spindexer.SpindexerKrakenSpinSpeed.get();
    }

    @Override
    public void periodic() {
        if (Settings.Spindexer.isSpindexerEnabled.getAsBoolean()) {
            spindexerKrakenMotor.setControl(new DutyCycleOut(getTargetSpeed().get()));
        }

        SmartDashboard.putNumber("Spindexer/ Kraken/ Target Speed (State Fraction)", getTargetSpeed().get());
        SmartDashboard.putNumber("Spindexer/ Kraken/ Actual Speed (Motor Duty Cycle)", getCurrentDutyCycle());
        //SmartDashboard.putNumber("Spindexer/ Kraken Voltage", spindexerKrakenMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Spindexer/ Kraken/ Current", spindexerKrakenMotor.getSupplyCurrent().getValueAsDouble());

        if (Settings.Spindexer.debugMode.getAsBoolean()) {
            SmartDashboard.putBoolean("Spindexer/ Kraken/ at target dutyCycle? (DEBUG)", getCurrentDutyCycle() >= 0.08);
            SmartDashboard.putNumber("Spindexer/ Kraken/ Setting value (DEBUG)", getSettingValue().get());
        }
    }
}   
