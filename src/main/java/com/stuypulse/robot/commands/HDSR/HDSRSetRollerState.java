package com.stuypulse.robot.commands.hdsr;

import com.stuypulse.robot.subsystems.hdsr.HoodedShooter;
import com.stuypulse.robot.subsystems.hdsr.HoodedShooter.RollerState;
import com.stuypulse.robot.subsystems.hdsr.HoodedShooter.ShooterState;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HDSRSetRollerState extends InstantCommand {
    private final HoodedShooter hdsr;
    private RollerState state;

    public HDSRSetRollerState(RollerState state) {
        hdsr = HoodedShooter.getInstance();
        this.state = state;
    }

    @Override
    public void execute() {
        hdsr.setRollerState(state);
    }

}
