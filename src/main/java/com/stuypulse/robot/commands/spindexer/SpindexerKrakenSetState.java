package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.spindexerKraken.SpindexerKraken;
import com.stuypulse.robot.subsystems.spindexerKraken.SpindexerKraken.SpindexerKrakenState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SpindexerKrakenSetState extends InstantCommand{
    private SpindexerKrakenState state;
    private final SpindexerKraken spindexerKraken;

    public SpindexerKrakenSetState(SpindexerKrakenState state) {
        this.state = state;
        spindexerKraken = SpindexerKraken.getInstance();

        addRequirements(spindexerKraken);
    }

    @Override
    public void initialize() {
        spindexerKraken.setKrakenState(this.state);
    }

}
