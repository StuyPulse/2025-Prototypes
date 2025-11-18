package com.stuypulse.robot.subsystems.HDSR;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class HoodedShooter extends SubsystemBase{
    public static final HoodedShooter instance;

    private HoodState state;
    private SmartNumber distanceToTarget;

    static {
        instance = new HoodedShooterImpl();
    }

    public static HoodedShooter getInstance(){
        return instance;
    }

    public enum HoodState{
        STOW(0.0),
        SHOOT(2000),
        DEFAULT(0.0);

        private double targetRPM;


        private HoodState(double targetRPM) {
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
        state = HoodState.STOW;
        distanceToTarget = new SmartNumber("HDSR/distanceToTarget", 0.0);
    }

    public HoodState getState() { return state; }

    public void setHoodState(HoodState state) {
        this.state = state;
    }

    public abstract double RPMToDistanceInterpolation(double distanceMeters);

    public double getDistanceToTarget() {
        return distanceToTarget.getAsDouble();
    }
   

    @Override
    public void periodic() {
        SmartDashboard.putString("HDSR/State", state.toString());
    }



}
