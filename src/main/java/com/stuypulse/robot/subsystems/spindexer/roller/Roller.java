package com.stuypulse.robot.subsystems.spindexer.roller;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Roller extends SubsystemBase{
    private static RollerImpl instance;
    private RollerState state;

    static {
        instance = new RollerImpl();
    }

    public static RollerImpl getInstance() {
        return instance;
    }

    public Roller() {
        state = RollerState.STOP;
    }

    public enum RollerState {
        SPIN(Settings.Spindexer.Roller.ROLLER_SPEED),
        STOP(0.0);

        private double speed;

        private RollerState(SmartNumber speed) {
            this.speed = speed.doubleValue();
        }

        private RollerState(double speed) {
            this.speed = speed;
        }

        public double getSpeed() {
            return speed;
        }
    }

    public RollerState getState() {
        return this.state;
    }

    public void setState(RollerState state) {
        this.state = state;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Subsystems/Spindexer/Roller/State", this.getState().toString());
    }
}
