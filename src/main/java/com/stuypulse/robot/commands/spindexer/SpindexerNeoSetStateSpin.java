package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.spindexerNeo.SpindexerNeo.SpindexerNeoState;

public class SpindexerNeoSetStateSpin extends SpindexerNeoSetState{
    public SpindexerNeoSetStateSpin() {
        super(SpindexerNeoState.SPIN);
    }
}
