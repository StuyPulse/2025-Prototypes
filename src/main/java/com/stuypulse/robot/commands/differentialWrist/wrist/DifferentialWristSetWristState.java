package com.stuypulse.robot.commands.differentialWrist.wrist;

import com.stuypulse.robot.subsystems.differentialWrist.DifferentialWrist;
import com.stuypulse.robot.subsystems.differentialWrist.DifferentialWrist.WristState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public abstract class DifferentialWristSetWristState extends InstantCommand {
    private final DifferentialWrist differentialWrist;
    private final WristState state;

    public DifferentialWristSetWristState(WristState state) {
        this.state = state;
        differentialWrist = DifferentialWrist.getInstance();
        addRequirements(differentialWrist);
    }

    @Override
    public void initialize() {
        differentialWrist.setWristState(state);
    }
}
