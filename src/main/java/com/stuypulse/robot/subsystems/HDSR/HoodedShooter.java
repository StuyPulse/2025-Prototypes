package com.stuypulse.robot.subsystems.HDSR;

import java.security.PublicKey;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class HoodedShooter extends SubsystemBase{
    public static final HoodedShooter instance;

    private ShooterState shooterState;
    private HoodState hoodState;
    private RollerState rollerState;

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
        DEFAULT(Constants.HDSR.HoodAngles.DEFAULT),
        EXTENDED(Constants.HDSR.HoodAngles.EXTENDED),
        STOW(Constants.HDSR.HoodAngles.STOW);

        private Rotation2d targetAngle;

        private HoodState(Rotation2d targetAngle) {
            this.targetAngle = targetAngle;
        }

        public Rotation2d getTargetAngle() {
            return targetAngle;
        }

    }

    public enum RollerState {
        INTAKE(Settings.HDSR.Roller.intakeSpeed),
        STOW(Settings.HDSR.Roller.stowSpeed),
        SHOOT(Settings.HDSR.Roller.ShootSpeed);

        private double targetAngle;

        private RollerState(double targetAngle) {
            this.targetAngle = targetAngle;
        }

        public double getTargetAngle() {
            return targetAngle;
        }
        
    }

    public enum ShooterState{
        STOW(0.0),
        SHOOTDEFAULT(2000),
        SHOOT(HoodedShooter.getInstance().RPMToDistanceInterpolation());

        private double targetRPM;

        private ShooterState(double targetRPM) {
            this.targetRPM = targetRPM;
        }

        public double getTargetRPM() {
            return targetRPM;
        }
    }

    public HoodedShooter() {
        shooterState = ShooterState.STOW;
        rollerState = RollerState.STOW;
        hoodState = HoodState.STOW;
    }


    public ShooterState getShooterState() { return shooterState; }

    public HoodState getHoodState() { return hoodState; }

    public RollerState getRollerState() { return rollerState; }

    public void setHoodState(HoodState state) {
        this.hoodState = state;
    }
    public void setRollerState(RollerState state) {
        this.rollerState = state;
    }

    public void setShooterState(ShooterState state) {
        this.shooterState = state;
    }

    public abstract double RPMToDistanceInterpolation();

    // public abstract boolean hasBall();

    public double getDistanceToTarget() {
        return 0.0;
    }
   

    @Override
    public void periodic() {
        SmartDashboard.putString("HDSR/ Shooter State", shooterState.toString());
        SmartDashboard.putString("HDSR/ Roller State", rollerState.toString());
        SmartDashboard.putString("HDSR/ Hood State", hoodState.toString());
    }



}
