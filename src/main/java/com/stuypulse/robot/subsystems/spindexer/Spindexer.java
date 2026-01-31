package com.stuypulse.robot.subsystems.spindexer;

import com.stuypulse.robot.subsystems.spindexer.roller.Roller;
import com.stuypulse.robot.subsystems.spindexer.roller.Roller.RollerState;
import com.stuypulse.robot.subsystems.spindexer.spinner.Spinner;
import com.stuypulse.robot.subsystems.spindexer.spinner.Spinner.SpinnerState;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
    private static final Spindexer instance;

    static {
        instance = new Spindexer();
    }

    public static Spindexer getInstance() {
        return instance;
    }

    public enum SpindexerState {
        STOP(RollerState.STOP, SpinnerState.STOP),
        RUN_ROLLER(RollerState.SPIN, SpinnerState.STOP),
        RUN_SPINNER(RollerState.STOP, SpinnerState.SPIN),
        RUN_ALL(RollerState.SPIN, SpinnerState.SPIN);

        private RollerState rollerState;
        private SpinnerState spinnerState;

        private SpindexerState(RollerState rollerState, SpinnerState spinnerState) {
            this.rollerState = rollerState;
            this.spinnerState = spinnerState;
        }

        public RollerState getRollerState() {
            return this.rollerState;
        }

        public SpinnerState getSpinnerState() {
            return this.spinnerState;
        }
    }

    private SpindexerState state;

    private final Roller roller;
    private final Spinner spinner;

    public Spindexer() {
        this.state = SpindexerState.STOP;
        this.roller = Roller.getInstance();
        this.spinner = Spinner.getInstance();
    }

    public void setState(SpindexerState state) {
        this.state = state;
        roller.setState(state.getRollerState());
        spinner.setState(state.getSpinnerState());
    }

    public SpindexerState getState() {
        return this.state;
    }

}
