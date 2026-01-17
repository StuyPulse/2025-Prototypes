package com.stuypulse.robot.subsystems.shooter;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    public static final Shooter instance;

    static { // singleton
        instance = new ShooterImpl();
    }

    public static Shooter getInstance() {
        return instance; 
    }

    public enum ShooterState {
        STOP(0),
        SHOOT(Settings.Shooter.SHOOTER_SHOOT);

        public double shooterState;

        private ShooterState(double shooterState) {
            this.shooterState = shooterState;
        }

        public double getSpeed() {
            return shooterState;
        }
    }

    private ShooterState state;

    public void setShooterState(ShooterState shooterState) {
        this.state = shooterState;
    }

    protected Shooter() {
        this.state = ShooterState.STOP;
    }

    public ShooterState getState() {
        return this.state;
    }

    @Override 
    public void periodic() {
        SmartDashboard.putString("ShooterState", getState().toString());
    }
}