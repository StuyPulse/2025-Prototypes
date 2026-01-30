package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SpindexerSetState extends InstantCommand{
    private SpindexerState state;
    private final Spindexer spindexer = new Spindexer();

    public SpindexerSetState(SpindexerState state) {
        this.state = state;
    }

    @Override
    public void initialize() {
        spindexer.setState(this.state);
    }

}
