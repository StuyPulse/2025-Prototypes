package com.stuypulse.robot.subsystems.hdsr;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class HoodedShooter extends SubsystemBase{
    public static final HoodedShooter instance;

    private ShooterState shooterState;
    private RollerState rollerState;

    static {
        instance = new HoodedShooterImpl();
    }

    public static HoodedShooter getInstance(){
        return instance;
    }

    public enum RollerState{
        STOW(0.0),
        Intake(2000), 
        RUNRPM(0.0);

        private double rollerRPM;

        private RollerState(double rollerRPM) {
            this.rollerRPM = rollerRPM;
        }

        public void setRollerRPM(double rollerRPM) {
            this.rollerRPM = rollerRPM;
        }

        public double getRollerRPM() {
            return rollerRPM;
        }
    }

    public enum ShooterState{
        STOW(0.0),
        SHOOTRPM(2000),
        LEVELINTERP(0.0),
        GOALINTERP(0.0);

        private double shooterRPM;

        private ShooterState(double targetRPM) {
            this.shooterRPM = targetRPM;
        }

        public void setShooterRPM(double targetRPM) {
            this.shooterRPM = targetRPM;
        }

        public double getShooterRPM() {
            return shooterRPM;
        }


    }

    public HoodedShooter() {
        shooterState = ShooterState.STOW;
        rollerState = RollerState.STOW;
    }

    public ShooterState getShooterState() { return shooterState; }

    public RollerState getRollerState() { return rollerState; }

    public void setShooterState(ShooterState state) {
        this.shooterState = state;
    }

    public void setRollerState(RollerState state) {
        this.rollerState = state;
    }

    public abstract double getShootDistanceRPM();

    public abstract void setTargetTranslation(Translation2d targetTranslation);

    public abstract double getShootGoalRPM();
   

    @Override
    public void periodic() {
        SmartDashboard.putString("HDSR/State", shooterState.toString());
    }
}
