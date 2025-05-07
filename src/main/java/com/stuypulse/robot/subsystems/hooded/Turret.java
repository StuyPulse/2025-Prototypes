package com.stuypulse.robot.subsystems.hooded;

import com.stuypulse.robot.constants.Ports;

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

    private double targetAngle = 0.0;

    public void setTargetAngle(double angle){
        targetAngle = angle;
    }
    public double getTargetAngle(){
        return targetAngle;
    }

    public abstract double getAngle();

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Turret/Target Angle", getTargetAngle());
    }
}