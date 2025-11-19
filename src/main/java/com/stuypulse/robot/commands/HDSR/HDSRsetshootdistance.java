package com.stuypulse.robot.commands.HDSR;

import java.util.Dictionary;

import com.stuypulse.robot.subsystems.HDSR.HoodedShooter;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HDSRsetshootdistance extends InstantCommand {
    private final Odometry odometry;
    private final HoodedShooter hdsr; 
    private double distance;

    public HDSRsetshootdistance(double distance) {
        odometry = Odometry.getInstance();
        hdsr = HoodedShooter.getInstance();
        this.distance = distance;
        
        addRequirements(hdsr);
    }

    @Override 
    public void execute() {
        hdsr.setTargetTranslation(new Translation2d(odometry.getPose().getX() + distance, odometry.getPose().getY()));
    }
    
}
