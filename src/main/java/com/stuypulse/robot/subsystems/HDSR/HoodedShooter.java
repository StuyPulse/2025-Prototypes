package com.stuypulse.robot.subsystems.HDSR;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class HoodedShooter extends SubsystemBase{
    public static final HoodedShooter instance;

    private HoodState state;

    static {
        instance = new HoodedShooterImpl();
        // if (Robot.isReal()){
        //     instance = new HoodedShooterImpl();
        // }
        // else {
        //     instance = new HoodedShooterSim();
        // }
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
    }

    public HoodState getState() { return state; }

    public void setHoodState(HoodState state) {
        this.state = state;
    }

    public abstract double RPMToDistanceInterpolation(double distanceMeters);
    // public abstract boolean hasBall();

    public double getDistanceToTarget() {
        return 0.0;
    }
   

    @Override
    public void periodic() {
        SmartDashboard.putString("HDSR/State", state.toString());
    }



}
