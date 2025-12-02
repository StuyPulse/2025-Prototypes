package com.stuypulse.robot.subsystems.HDSR;

import java.security.PublicKey;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.HDSR.Roller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class HoodedShooter extends SubsystemBase{
    public static final HoodedShooter instance;

    private ShooterState shooterState;
    private HoodState hoodState;
    private RollerState rollerState;
    private HDSRState hdsrState;

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
        return new HoodedShooterImpl();
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
        SHOOT(0);

        private double targetRPM;

        private ShooterState(double targetRPM) {
            this.targetRPM = targetRPM;
        }

        public double getTargetRPM() {
            return targetRPM;
        }
    }

    public enum HDSRState{
        STOW(HoodState.STOW, ShooterState.STOW, RollerState.STOW),
        SHOOT(HoodState.EXTENDED, ShooterState.SHOOT, RollerState.SHOOT),
        INTAKE(HoodState.DEFAULT, ShooterState.STOW, RollerState.INTAKE);
        
        private HoodState hoodState;
        private ShooterState shooterState;
        private RollerState rollerState;
        
        private HDSRState(HoodState hoodState, ShooterState shooterState, RollerState rollerState) {
            this.shooterState = shooterState;
            this.hoodState = hoodState;
            this.rollerState = rollerState;
        }

        public HoodState getHoodState() {
            return hoodState;
        }

        public ShooterState getShooterState() {
            return shooterState;
        }

        public RollerState getRollerState() {
            return rollerState;
        }
    }

    public HoodedShooter() {
         hdsrState = HDSRState.STOW;
         shooterState = ShooterState.STOW;
         rollerState = RollerState.STOW;
         hoodState = HoodState.STOW;
    }


    public ShooterState getShooterState() { return shooterState; }

    public HoodState getHoodState() { return hoodState; }

    public RollerState getRollerState() { return rollerState; }

    public HDSRState getHDSRState() {return hdsrState; } 


    public void setHDSRstate(HDSRState hdsrState) {
        this.hdsrState = hdsrState;
        this.shooterState = hdsrState.getShooterState();
        this.rollerState = hdsrState.getRollerState();
        this.hoodState = hdsrState.getHoodState();
    }

    // public abstract boolean hasBall();

    public double getDistanceToTarget() {
        return 0.0;
    }
   

    @Override
    public void periodic() {
        SmartDashboard.putString("HDSR/ Shooter State", shooterState.toString());
        SmartDashboard.putString("HDSR/ Roller State", rollerState.toString());
        SmartDashboard.putString("HDSR/ Hood State", hoodState.toString());
        SmartDashboard.putString("HDSR/ HDSR state", hdsrState.toString());
    }



}
