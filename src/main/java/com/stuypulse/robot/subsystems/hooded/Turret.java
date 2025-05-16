package com.stuypulse.robot.subsystems.hooded;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Turret extends SubsystemBase {

    public static final Turret instance;
    static {
        if (Robot.isReal()){
            instance = new TurretImpl();
        } else {
            instance = new TurretSim();
        }
    }

    public static Turret getInstance(){
        return instance;
    }

    public enum TurretState {
        INTAKE_SIDE(Settings.Turret.States.INTAKE_SIDE),
        SHOOTER_SIDE(Settings.Turret.States.SHOOTER_SIDE),
        FREE;

        private Rotation2d targetAngle;

        private TurretState(Rotation2d angle) {
            targetAngle = angle;
        }

        private TurretState(){
            
        }

        public Rotation2d getTargetAngle() {
            return targetAngle;
        }
    }

    private TurretState state;
    private Rotation2d targetAngle;
    private TurretVisualizer visualizer;

    protected Turret() {
        state = TurretState.FREE;
        targetAngle = new Rotation2d();
        visualizer = new TurretVisualizer();
    }

    public TurretState getState(){
        return state;
    }

    public void setState(TurretState turretState){
        state = turretState;
    }
    
    public void setTargetAngle(Rotation2d angle){
        targetAngle = angle;
    }
    
    public Rotation2d getTargetAngle(){
        if (getState() == TurretState.FREE) {
            return targetAngle;
        } else {
            return getState().getTargetAngle();
        }
    }

    public abstract Rotation2d getAngle();
    public abstract boolean atTargetAngle();

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Turret/Target Angle", getTargetAngle().getDegrees());
        SmartDashboard.putNumber("Turret/Turret Angle", getAngle().getDegrees());

        // add debug later prob
        if (true){
            visualizer.updateAngle(getAngle());
        }
    }
}