package com.stuypulse.robot.subsystems.spindexer;

import com.stuypulse.robot.constants.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase{
    private static SpindexerImpl instance;
    private SpindexerState state;

    static {
        instance = new SpindexerImpl();
    }

    public static SpindexerImpl getInstance() {
        return instance;
    }

    public Spindexer() {
        state = SpindexerState.STOP;
    }

    public enum SpindexerState {
        SPIN(Constants.Spindexer.SpindexerSpinSpeed),
        STOP(Constants.Spindexer.SpindexerStopSpeed);

        private double speed;

        private SpindexerState(double speed) {
            this.speed = speed;
        }

        public double getSpindexerSpeed() {
            return speed;
        }
    }

    public SpindexerState getState() {
        return this.state;
    }

    public void setState(SpindexerState state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("SUBSYSTEMTS/SPINDEXER", "SPINDEXER");
    }
}
