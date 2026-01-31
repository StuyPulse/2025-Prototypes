package com.stuypulse.robot.subsystems.spindexerNeo;


import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerNeo extends SubsystemBase{
    private static SpindexerNeo instance;

    static {
        instance = new SpindexerNeoImpl();
    }

    public static SpindexerNeo getInstance() {
        return instance;
    }

    public enum SpindexerNeoState {
        SPIN(Settings.Spindexer.SpindexerNeoSpinSpeed),
        STOP(Settings.Spindexer.SpindexerNeoStopSpeed);

        private double speed;

        private SpindexerNeoState(double speed) {
            this.speed = speed;
        }

        public double getNeoSpindexerSpeed() {
            return this.speed;
        }
    }

    private SpindexerNeoState state;

    public SpindexerNeo() {
        this.state = SpindexerNeoState.STOP;
    }

    public SpindexerNeoState getNeoState() {
        return this.state;
    }

    public void setNeoState(SpindexerNeoState state) {
        this.state = state;
    }
    @Override
    public void periodic() {
        SmartDashboard.putString("Spindexer/Neo State", getNeoState().toString());
    }
}
