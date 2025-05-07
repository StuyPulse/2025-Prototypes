package com.stuypulse.robot.subsystems.differentialWrist;

import com.stuypulse.robot.Robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DifferentialWrist extends SubsystemBase {
    public static final DifferentialWrist instance;

    
    public enum WristStates{

    }

    public enum RollerStates{
        
    }

    public DifferentialWrist(){
        
    } 


    static {
        if(Robot.isReal()) {
            instance = new DifferentialWristImpl();
        } else {
            instance = new DifferentialWristSim();
        }
    }

    public static DifferentialWrist getInstance() {
        return instance;
    }

    public abstract Rotation2d getCurrentPitchAngle();

    public abstract Rotation2d getCurrentRollAngle();

}
