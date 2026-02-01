package com.stuypulse.robot.subsystems.spindexerKraken;


import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpindexerKraken extends SubsystemBase{
    private static SpindexerKraken instance;
    private static SpindexerKrakenState state;

    //private static Supplier<Double> spin = () -> Settings.Spindexer.SpindexerKrakenSpinSpeed.get();

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
        SPIN(() -> Settings.Spindexer.SpindexerKrakenSpinSpeed.get()),
        STOP(() -> 0.0);
        private Supplier<Double> speed; //might convert to supplier too

        private SpindexerKrakenState(Supplier<Double> speed) {
            this.speed = speed;
        }

        public Supplier<Double> getSpindexerSpeed() {
            return speed;
        }
    }

    public static SpindexerKrakenState getKrakenState() {
        return state;
    }

    public void setKrakenState(SpindexerKrakenState state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Spindexer/Kraken/ State", getKrakenState().toString());
    }
}
