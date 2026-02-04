package com.stuypulse.robot.subsystems.spindexerNeo;


import java.util.function.Supplier;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.SpindexerSim;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerNeo extends SubsystemBase{
    private static SpindexerNeo instance;

    static {
        if (Robot.isReal()) {
            instance = new SpindexerNeoImpl();
        }
        else  {
            instance = new SpindexerSim();
        }
    }

    public static SpindexerNeo getInstance() {
        return instance;
    }

    public enum SpindexerNeoState {
        SPIN(() -> Settings.Spindexer.SpindexerNeoSpinSpeed.get()),
        STOP(() -> 0.0);

        private Supplier<Double> speed;

        private SpindexerNeoState(Supplier<Double> speed) {
            this.speed = speed;
        }

        public Supplier<Double> getNeoSpindexerSpeed() {
            return this.speed;
        }
    }

    private static SpindexerNeoState state;

    public SpindexerNeo() {
        this.state = SpindexerNeoState.STOP;
    }

    public static SpindexerNeoState getNeoState() {
        return state;
    }

    public void setNeoState(SpindexerNeoState state) {
        this.state = state;
    }
    @Override
    public void periodic() {
        SmartDashboard.putString("Spindexer/Neo/ State", getNeoState().toString());
    }
}
