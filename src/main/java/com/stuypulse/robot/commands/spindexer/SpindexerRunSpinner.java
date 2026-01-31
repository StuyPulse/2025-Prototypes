package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;

public class SpindexerRunSpinner extends SpindexerSetState {
    public SpindexerRunSpinner() {
        super(SpindexerState.RUN_SPINNER);
    }
}
