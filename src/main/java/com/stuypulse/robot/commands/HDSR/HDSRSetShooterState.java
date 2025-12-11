package com.stuypulse.robot.commands.hdsr;

import com.stuypulse.robot.subsystems.hdsr.HoodedShooter;
import com.stuypulse.robot.subsystems.hdsr.HoodedShooter.ShooterState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HDSRSetShooterState extends InstantCommand {
    private final HoodedShooter hdsr;
    private ShooterState state;

    public HDSRSetShooterState(ShooterState state) {
        hdsr = HoodedShooter.getInstance();
        this.state = state;

        addRequirements(hdsr);
    }

    @Override
    public void execute() {
        hdsr.setShooterState(state);
    }

    @Override
    public boolean isFinished() {
        return hdsr.getShooterState() == state;
    }
}
