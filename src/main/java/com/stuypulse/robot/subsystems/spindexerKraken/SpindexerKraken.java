package com.stuypulse.robot.subsystems.spindexerKraken;


import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerKraken extends SubsystemBase{
    private static SpindexerKraken instance;
    private SpindexerKrakenState state;

    static {
        instance = new SpindexerKrakenImpl();
    }

    public static SpindexerKraken getInstance() {
        return instance;
    }

    public SpindexerKraken() {
        state = SpindexerKrakenState.STOP;
    }

    public enum SpindexerKrakenState {
        SPIN(Settings.Spindexer.SpindexerKrakenSpinSpeed),
        STOP(Settings.Spindexer.SpindexerKrakenStopSpeed);

        private double speed;

        private SpindexerKrakenState(double speed) {
            this.speed = speed;
        }

        public double getSpindexerSpeed() {
            return speed;
        }
    }

    public SpindexerKrakenState getKrakenState() {
        return this.state;
    }

    public void setKrakenState(SpindexerKrakenState state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Spindexer/Kraken State", getKrakenState().toString());
    }
}
