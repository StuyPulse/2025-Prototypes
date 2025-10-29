package com.stuypulse.robot.subsystems.hdsr;

import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;

public class HoodedShooterSim {

    private AnglePIDController hoodController;
    private PIDController shooterController;

    // we use da flywheel sim probably
    public HoodedShooterSim(){
        super();
    }
}
