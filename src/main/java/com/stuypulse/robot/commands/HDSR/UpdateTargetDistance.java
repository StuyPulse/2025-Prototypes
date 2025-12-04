package com.stuypulse.robot.commands.HDSR;

import java.util.Dictionary;

import javax.crypto.spec.RC2ParameterSpec;

import com.stuypulse.robot.subsystems.HDSR.HoodedShooter;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class UpdateTargetDistance extends InstantCommand{
    private final HoodedShooter hdsr;
    private double distance;
    
    public UpdateTargetDistance(double distance) {
        hdsr = HoodedShooter.getInstance();
        this.distance = distance;
        addRequirements(hdsr);
    }

    @Override 
    public void execute() {
        hdsr.UpdateTargetDistance(distance);
    }
}
