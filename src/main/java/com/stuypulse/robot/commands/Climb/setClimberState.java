package com.stuypulse.robot.commands.Climb;

import com.stuypulse.robot.subsystems.CLIMBER.Climber;
import com.stuypulse.robot.subsystems.CLIMBER.Climber.ClimberState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class setClimberState extends InstantCommand {
    private final Climber climber;
    private ClimberState state;

    public setClimberState(ClimberState state) {
        this.climber = Climber.getInstance();
        this.state = state;
    }

    @Override
    public void execute() {
        climber.setClimberState(state);
    }
}
