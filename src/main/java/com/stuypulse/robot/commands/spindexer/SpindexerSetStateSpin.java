package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;

public class SpindexerSetStateSpin extends SpindexerSetState {
    public SpindexerSetStateSpin() {
        super(SpindexerState.SPIN);
    }
}
