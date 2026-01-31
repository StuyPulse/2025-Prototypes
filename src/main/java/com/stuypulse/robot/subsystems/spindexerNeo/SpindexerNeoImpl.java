package com.stuypulse.robot.subsystems.spindexerNeo;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpindexerNeoImpl extends SpindexerNeo{
    SparkMax SpindexerNeo;

    public SpindexerNeoImpl() {
        SpindexerNeo = new SparkMax(Ports.Spindexer.SPINDEXER_NEO_MOTOR, MotorType.kBrushless); //update with port
        
        SpindexerNeo.configure(Motors.Spindexer.SPINDEXER_NEO_MOTOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SpindexerNeo.set(getNeoState().getNeoSpindexerSpeed());

        SmartDashboard.putNumber("Spindexer/ Neo Speed", getNeoState().getNeoSpindexerSpeed());

       SmartDashboard.putNumber("Spindexer/ Neo get Current", SpindexerNeo.getOutputCurrent()); 
    }  
}
