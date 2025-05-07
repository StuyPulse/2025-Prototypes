package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase{
    public static final Arm instance;

    static {
        if (Robot.isReal()){
            instance = new ArmImpl();
        } else {
            instance = new ArmSimu();
        }
    }

    public static Arm getInstance(){
        return instance;
    }
    
    public abstract double getShoulderAngle();
    public abstract double getElbowAngle();


    public enum ArmState{
    // unfinished

    }


}
