package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.spindexerNeo.SpindexerNeo.SpindexerNeoState;

public class SpindexerNeoSetStateStop extends SpindexerNeoSetState{
    public SpindexerNeoSetStateStop() {
        super(SpindexerNeoState.STOP);
    }
}