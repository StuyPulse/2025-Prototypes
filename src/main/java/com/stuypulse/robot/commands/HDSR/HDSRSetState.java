package com.stuypulse.robot.commands.HDSR;

import com.stuypulse.robot.subsystems.wiffleWorksShooter.WiffleWorksShooter;
import com.stuypulse.robot.subsystems.wiffleWorksShooter.WiffleWorksShooter.ShooterState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HDSRSetState extends InstantCommand{
    private ShooterState state;
    private WiffleWorksShooter hdsr;

    public HDSRSetState(ShooterState hoodState){
        state = hoodState;
        hdsr = WiffleWorksShooter.getInstance();

        addRequirements(hdsr);
    }

    @Override
    public void initialize(){
        hdsr.setHoodState(state);
    }
}
