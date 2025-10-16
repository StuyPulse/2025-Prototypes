package com.stuypulse.robot.commands.HDSR;

import com.stuypulse.robot.subsystems.HDSR.HoodedShooter;

import edu.wpi.first.wpilibj2.command.Command;
import com.stuypulse.robot.constants.Settings;


public class HDSRDefaultCommand extends Command{
    private HoodedShooter hdsr;

    public HDSRDefaultCommand(){
        hdsr = HoodedShooter.getInstance();
        
        addRequirements(hdsr);
    }

    @Override
    public void execute(){
        if (hdsr.hasBall()) {
            hdsr.setRollerSpeeds(Settings.HDSR.ROLLER_SPEED);

        }
        else hdsr.setRollerSpeeds(0.0);    }
}