package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.spindexerNeo.SpindexerNeo;
import com.stuypulse.robot.subsystems.spindexerNeo.SpindexerNeo.SpindexerNeoState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SpindexerNeoSetState extends InstantCommand {
    private final SpindexerNeo spindexerNeo;
    private SpindexerNeoState state;

    public SpindexerNeoSetState(SpindexerNeoState state) {
        spindexerNeo = SpindexerNeo.getInstance();
        
        this.state = state;

        addRequirements(spindexerNeo);
    }

    @Override
    public void initialize() {
        spindexerNeo.setNeoState(state);
    }

    

}
