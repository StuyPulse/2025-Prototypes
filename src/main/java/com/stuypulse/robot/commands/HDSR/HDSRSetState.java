package com.stuypulse.robot.commands.hdsr;

import com.stuypulse.robot.subsystems.hdsr.HoodedShooter;
import com.stuypulse.robot.subsystems.hdsr.HoodedShooter.ShooterState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HDSRSetState extends InstantCommand{
    private ShooterState state;
    private HoodedShooter hdsr;

    public HDSRSetState(ShooterState hoodState){
        state = hoodState;
        hdsr = HoodedShooter.getInstance();

        addRequirements(hdsr);
    }

    @Override
    public void initialize(){
        hdsr.setShooterState(state);
    }
}
