package com.stuypulse.robot.subsystems.shooter;

import java.util.function.Supplier;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    public static final Shooter instance;
    private SmartNumber speed;

    static { // singleton
        instance = new ShooterImpl();
    }

    public static Shooter getInstance() {
        return instance; 
    }

    private ShooterState state;

    public void setShooterState(ShooterState shooterState) {
        this.state = shooterState;
    }

    protected Shooter() {
        this.state = ShooterState.STOP;
        speed = new SmartNumber("Shooter Speed", Settings.Shooter.SHOOTER_SHOOT);
    }

    public enum ShooterState {
        STOP,
        SHOOT;
    }

    public Supplier<Double> getShooterSpeed() {
        return switch (getState()) {
            case STOP -> () -> 0.0;
            case SHOOT -> () -> speed.get();
        };
    }

    public ShooterState getState() {
        return this.state;
    }

    @Override 
    public void periodic() {
        SmartDashboard.putString("ShooterState", getState().toString());
        // SmartDashboard.putNumber("Current Speed", getShooterSpeed().get());
    }
}