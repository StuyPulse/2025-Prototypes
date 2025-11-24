package com.stuypulse.robot.subsystems.wiffleWorksShooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class WiffleWorksShooter extends SubsystemBase{
    public static final WiffleWorksShooter instance;

    private ShooterState state;

    static {
        instance = new WiffleWorksShooterImpl();
    }

    public static WiffleWorksShooter getInstance(){
        return instance;
    }

    public enum ShooterState{
        STOW(0.0),
        SHOOT(2000),
        DEFAULT(0);

        private double targetRPM;

        private ShooterState(double targetRPM) {
            this.targetRPM = targetRPM;
        }

        public double getTargetRPM() {
            return targetRPM;
        }
        public void setTargetRPM(double targetRPM) {
            this.targetRPM = targetRPM;
        }
    }

    public WiffleWorksShooter() {
        state = ShooterState.STOW;
    }

    public ShooterState getState() { return state; }

    public void setHoodState(ShooterState state) {
        this.state = state;
    }

    public abstract void setTargetTranslation(Translation2d targetTranslation);

    public abstract double RPMToDistanceInterpolation();

    public double getDistanceToTarget() {
        return 0.0;
    }
   

    @Override
    public void periodic() {
        SmartDashboard.putString("HDSR/State", state.toString());
    }



}
