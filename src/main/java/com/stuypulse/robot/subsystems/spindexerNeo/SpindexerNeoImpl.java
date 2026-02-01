package com.stuypulse.robot.subsystems.spindexerNeo;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpindexerNeoImpl extends SpindexerNeo{
    SparkMax SpindexerNeo;

    public SpindexerNeoImpl() {
        SpindexerNeo = new SparkMax(Ports.Spindexer.SPINDEXER_NEO_MOTOR, MotorType.kBrushless); //update with port
        
        SpindexerNeo.configure(Motors.Spindexer.SPINDEXER_NEO_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public static Supplier<Double> getSettingValue() {
        return () -> Settings.Spindexer.SpindexerNeoSpinSpeed.get();
    }

    @Override
    public void periodic() {
        if (Settings.Spindexer.isSpindexerEnabled.getAsBoolean()) {
            SpindexerNeo.set(getNeoState().getNeoSpindexerSpeed().get());
        }

        SmartDashboard.putNumber("Spindexer/ Neo/ Target Speed (State Fraction) ", getNeoState().getNeoSpindexerSpeed().get());
        SmartDashboard.putNumber("Spindexer/ Neo/ Actual Speed (Fractional Speed) ", SpindexerNeo.get());
        SmartDashboard.putNumber("Spindexer/ Neo/ Current", SpindexerNeo.getOutputCurrent());

        if (Settings.Spindexer.debugMode.getAsBoolean()) {
            SmartDashboard.putNumber("Spindexer/ Neo/ Setting Value (DEBUG)", getSettingValue().get());
        }
    }  
}
