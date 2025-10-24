package com.stuypulse.robot.subsystems.HDSR;

import com.stuypulse.robot.Robot;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
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
        STOW(new Rotation2d(0), 0.0),
        SHOOT(new Rotation2d(10), 3000.0);

        private Rotation2d targetAngle;
        private double targetRPM;

        private HoodState(Rotation2d targetAngle, double targetRPM) {
            this.targetAngle = targetAngle;
            this.targetRPM = targetRPM;
        }

        public Rotation2d getTargetAngle(){
            return targetAngle;
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

    // public abstract boolean hasBall();
   

    @Override
    public void periodic() {
        SmartDashboard.putString("HDSR/State", state.toString());
    }



}
