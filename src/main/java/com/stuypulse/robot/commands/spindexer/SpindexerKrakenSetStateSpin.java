package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.spindexerKraken.SpindexerKraken.SpindexerKrakenState;

public class SpindexerKrakenSetStateSpin extends SpindexerKrakenSetState {
    public SpindexerKrakenSetStateSpin() {
        super(SpindexerKrakenState.SPIN);
    }
}
