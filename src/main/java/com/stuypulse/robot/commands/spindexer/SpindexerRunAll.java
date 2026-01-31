package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.spindexer.Spindexer.SpindexerState;

public class SpindexerRunAll extends SpindexerSetState {
    public SpindexerRunAll() {
        super(SpindexerState.RUN_ALL);
    }
}
