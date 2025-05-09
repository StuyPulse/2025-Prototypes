package com.stuypulse.robot.commands.differentialWrist.roller;

import com.stuypulse.robot.subsystems.differentialWrist.DifferentialWrist;
import com.stuypulse.robot.subsystems.differentialWrist.DifferentialWrist.RollerState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DifferentialWristSetRollerState extends InstantCommand {
    private final DifferentialWrist differentialWrist;
    private final RollerState state;

    public DifferentialWristSetRollerState(RollerState state) {
        this.state = state;
        differentialWrist = DifferentialWrist.getInstance();
        addRequirements(differentialWrist);
    }

    @Override
    public void initialize() {
        differentialWrist.setRollerState(state);
    }
}
