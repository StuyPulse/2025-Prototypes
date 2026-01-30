package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;

public class SpindexerSetStateStop extends SpindexerSetState {
    public SpindexerSetStateStop() {
        super(SpindexerState.STOP);
    }
}
