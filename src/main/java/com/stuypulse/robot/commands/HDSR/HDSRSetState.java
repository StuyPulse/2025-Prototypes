package com.stuypulse.robot.commands.HDSR;

import com.stuypulse.robot.subsystems.hdsr.HoodedShooter;
import com.stuypulse.robot.subsystems.hdsr.HoodedShooter.HoodState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HDSRSetState extends InstantCommand{
    private HoodState state;
    private HoodedShooter hdsr;

    public HDSRSetState(HoodState hoodState){
        state = hoodState;
        hdsr = HoodedShooter.getInstance();

        addRequirements(hdsr);
    }

    @Override
    public void initialize(){
        hdsr.setHoodState(state);
    }
}
