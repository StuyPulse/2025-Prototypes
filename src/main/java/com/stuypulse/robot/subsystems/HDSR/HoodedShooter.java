package com.stuypulse.robot.subsystems.HDSR;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class HoodedShooter extends SubsystemBase{
    public static final HoodedShooter instance;

    private ShooterState state;

    static {
        instance = new HoodedShooterImpl();
    }

    public static HoodedShooter getInstance(){
        return instance;
    }

    public enum ShooterState{
        STOW(0.0),
        SHOOTRPM(2000),
        INTERP(0.0);

        private double targetRPM;


        private ShooterState(double targetRPM) {
            this.targetRPM = targetRPM;
        }

        public void setTargetRPM(double targetRPM) {
            this.targetRPM = targetRPM;
        }

        public double getTargetRPM() {
            return targetRPM;
        }

    }

    public HoodedShooter() {
        state = ShooterState.STOW;
    }

    public ShooterState getState() { return state; }

    public void setShooterState(ShooterState state) {
        this.state = state;
    }

    public abstract double distanceInterpolation();

    public abstract void setTargetTranslation(Translation2d targetTranslation);

    public abstract void updateTargetDistance(double targetDistance);

    public abstract double getTargetDistance();
   

    @Override
    public void periodic() {
        SmartDashboard.putString("HDSR/State", state.toString());
    }



}
