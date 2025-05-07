package com.stuypulse.robot.subsystems.hooded;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Hood extends SubsystemBase{
    private static final Hood instance;
    static {
        // TODO: change ts later
        instance  = new HoodImpl();
    }
    
    public enum HoodState {
        
    }
}
