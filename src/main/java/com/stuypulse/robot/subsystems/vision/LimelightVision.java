package com.stuypulse.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class LimelightVision extends SubsystemBase{
    private static LimeLightVisionImpl instance;
    
    static {
        instance = new LimeLightVisionImpl();
    }

    public static LimeLightVisionImpl getInstance(){
        return instance;
    }
    
}
