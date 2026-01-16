package com.stuypulse.robot.subsystems.CLIMBER;

import com.stuypulse.robot.constants.*;

import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.stuylib.util.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ClimberImpl extends Climber {
    ClimberState state;

    SparkMax MOTOR_ONE; 
    SparkMax MOTOR_TWO;

   // SmartNumber ClimberCycle;

    public ClimberImpl() {
        super();

        state = getClimberState().STOW;
        MOTOR_ONE = new SparkMax(Ports.Climber.MOTOR_ONE, MotorType.kBrushless);
        MOTOR_TWO = new SparkMax(Ports.Climber.MOTOR_TWO, MotorType.kBrushless);

        MOTOR_ONE.configure(Motors.Climber.Intake_Motor_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        MOTOR_TWO.configure(Motors.Climber.Intake_Motor_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

       // ClimberCycle = new SmartNumber("Climber/Climber Voltage value", 0);
    }

    @Override
    public void periodic() {
        switch (getClimberState()) {
            case CLIMB: 
            state.setClimberCycle(1);
            break;

            case DOWN:
            state.setClimberCycle(-1);
            break;

            case STOW:
            state.setClimberCycle(0);
            break;

            default: 
            state.setClimberCycle(0);
            break;

        }

        MOTOR_ONE.setVoltage(state.getClimberCycle());
        MOTOR_TWO.setVoltage(state.getClimberCycle());

        

    }



}
