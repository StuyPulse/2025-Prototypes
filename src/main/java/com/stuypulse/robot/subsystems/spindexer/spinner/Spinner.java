package com.stuypulse.robot.subsystems.spindexer.spinner;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spinner extends SubsystemBase{
    private static SpinnerImpl instance;
    private SpinnerState state;

    static {
        instance = new SpinnerImpl();
    }

    public static SpinnerImpl getInstance() {
        return instance;
    }

    public Spinner() {
        state = SpinnerState.STOP;
    }

    public enum SpinnerState {
        SPIN(Settings.Spindexer.Spinner.SPINNER_SPEED),
        STOP(0.0);

        private double speed;

        private SpinnerState(SmartNumber speed) {
            this.speed = speed.doubleValue();
        }

        private SpinnerState(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return speed;
        }
    }

    public SpinnerState getState() {
        return this.state;
    }

    public void setState(SpinnerState state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Subsystems/Spindexer/Spinner/State", this.getState().toString());
    }
}
