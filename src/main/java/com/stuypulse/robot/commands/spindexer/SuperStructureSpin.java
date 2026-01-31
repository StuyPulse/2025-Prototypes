package com.stuypulse.robot.commands.spindexer;

import com.stuypulse.robot.subsystems.spindexerKraken.SpindexerKraken;
import com.stuypulse.robot.subsystems.spindexerKraken.SpindexerKraken.SpindexerKrakenState;
import com.stuypulse.robot.subsystems.spindexerKraken.SpindexerKrakenImpl;
import com.stuypulse.robot.subsystems.spindexerNeo.SpindexerNeo;
import com.stuypulse.robot.subsystems.spindexerNeo.SpindexerNeo.SpindexerNeoState;

import edu.wpi.first.wpilibj2.command.InstantCommand;



public class SuperStructureSpin extends InstantCommand {
    SpindexerKraken spindexerKraken;
    SpindexerKrakenState KrakenState;

    SpindexerNeo spindexerNeo;
    SpindexerNeoState NeoState;

    public SuperStructureSpin() {
        spindexerKraken = SpindexerKraken.getInstance();
        KrakenState = SpindexerKrakenState.SPIN;

        spindexerNeo = SpindexerNeo.getInstance();
        NeoState = SpindexerNeo.SpindexerNeoState.SPIN;

        addRequirements(spindexerKraken, spindexerNeo);
    }
    @Override
    public void execute() {
        spindexerKraken.setKrakenState(KrakenState);
        if (SpindexerKrakenImpl.atDutyCycle().get()) {
            spindexerNeo.setNeoState(NeoState);
        }
    }
}
