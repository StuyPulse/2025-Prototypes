package com.stuypulse.robot.subsystems.HDSR;

import com.stuypulse.robot.Robot;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class HoodedShooter extends SubsystemBase{
    public static final HoodedShooter instance;
    private SmartNumber targetVelocity;
    private SmartNumber targetAngle;
    private SmartBoolean hasBall;

    private HoodState state;


    static {
        if (Robot.isReal()){
            instance = new HoodedShooterImpl();
        } else {
            instance = new HoodedShooterSim();
        }
    }

    public static HoodedShooter getInstance(){
        return instance;
    }

    public enum HoodState{
        STOW(new Rotation2d(0), 0.0),
        SHOOT(new Rotation2d(10), 1.0);

        private Rotation2d hoodAngle;
        private double shooterVelocity;

        
        private HoodState(Rotation2d hoodAngle, double shooterVelocity) {
            this.hoodAngle = hoodAngle;
            this.shooterVelocity = shooterVelocity;
        }

        public Rotation2d getHoodAngle(){
            return hoodAngle;
        }

        public double getShooterVelocity() {
            return shooterVelocity;
        }

    }

    public HoodedShooter() {
        state = HoodState.STOW;
        hasBall = new SmartBoolean("HDSR/hasBall", false);
    }

    public void setHoodState(HoodState state) {
        this.state = state;
        targetAngle.set(state.getHoodAngle().getDegrees());
        targetVelocity.set(state.getShooterVelocity());
    }
    
    
    public Rotation2d getTargetAngle() {
        return Rotation2d.fromDegrees(targetAngle.doubleValue());
    }

    public double getTargetVelocity() {
        return targetVelocity.doubleValue();
    }

    public boolean hasBall(){
        return hasBall.get();
    }

    public abstract void setRollerSpeeds(double speed);
   

    @Override
    public void periodic() {
        SmartDashboard.putString("HDSR/state", state.toString());
    }



}
