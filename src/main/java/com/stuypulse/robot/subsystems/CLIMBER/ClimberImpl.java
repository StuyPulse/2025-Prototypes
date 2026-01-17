package com.stuypulse.robot.subsystems.CLIMBER;

import com.stuypulse.robot.constants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ClimberImpl extends Climber {
    Climber climber;

    SparkMax MOTOR_ONE; 
    SparkMax MOTOR_TWO;

   // SmartNumber ClimberCycle;

    public ClimberImpl() {
        super();

        MOTOR_ONE = new SparkMax(Ports.Climber.MOTOR_ONE, MotorType.kBrushless);
        MOTOR_TWO = new SparkMax(Ports.Climber.MOTOR_TWO, MotorType.kBrushless);

        MOTOR_ONE.configure(Motors.Climber.climbMotor_one, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        MOTOR_TWO.configure(Motors.Climber.climbMotor_two, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    @Override
    public void periodic() {

        MOTOR_ONE.setVoltage(getClimberState().getClimberVolts());
        MOTOR_TWO.setVoltage(getClimberState().getClimberVolts());

        SmartDashboard.putString("STATE", getClimberState().toString());
        SmartDashboard.putNumber("VOLTAGE", getClimberState().getClimberVolts());

    }



}
