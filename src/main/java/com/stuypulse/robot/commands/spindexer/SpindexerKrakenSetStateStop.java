package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.spindexerKraken.SpindexerKraken.SpindexerKrakenState;

public class SpindexerKrakenSetStateStop extends SpindexerKrakenSetState {
    public SpindexerKrakenSetStateStop() {
        super(SpindexerKrakenState.STOP);
    }
}
