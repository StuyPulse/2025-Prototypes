package com.stuypulse.robot.subsystems.hooded;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.hooded.Hood.HoodState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Turret extends SubsystemBase {

    public static final Turret instance;
    static {
        // TODO: change this later
        instance = new TurretImpl();
    }

    public Turret getInstance(){
        return instance;
    }

    public enum TurretState {
        INTAKE_SIDE(Settings.Turret.States.INTAKE_SIDE),
        SHOOTER_SIDE(Settings.Turret.States.SHOOTER_SIDE),
        FREE(0);

        private double targetAngle;

        private TurretState(double angle) {
            targetAngle = angle;
        }

        public double getTargetAngle() {
            return targetAngle;
        }
    }

    private TurretState state;
    private double targetAngle;

    protected Turret() {
        state = TurretState.FREE;
        targetAngle = 0;
    }

    public TurretState getState(){
        return state;
    }

    public void setState(TurretState turretState){
        state = turretState;
    }
    
    public void setTargetAngle(double angle){
        targetAngle = angle;
    }
    
    public double getTargetAngle(){
        if (getState() == TurretState.FREE) {
            return targetAngle;
        } else {
            return getState().getTargetAngle();
        }
    }

    public abstract double getAngle();
    public abstract boolean atTargetAngle();

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Turret/Target Angle", getTargetAngle());
        SmartDashboard.putNumber("Turret/Turret Angle", getAngle());
    }
}