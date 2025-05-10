package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase{
    public static final Arm instance;

    static {
        if (Robot.isReal()){
            instance = new ArmImpl();
        } else {
            instance = new ArmSim();
        }
    }

    public static Arm getInstance(){
        return instance;
    }

    public enum ArmState {
        //Give States with degrees or smth
        INTAKE(0,0);

        public final double shoulderDegrees;
        public final double elbowDegrees;

        ArmState(double shoulderDegrees, double elbowDegrees) {
            this.shoulderDegrees = shoulderDegrees;
            this.elbowDegrees = elbowDegrees;
        }
    }

    // Core methods
    public abstract double getShoulderAngleDegrees();
    public abstract double getElbowAngleDegrees();
    public abstract void setVoltages(double shoulderVolts, double elbowVolts);
    public abstract void resetEncoders();
    public abstract void stop();

    // High-level control
    public void setState(ArmState state) {
        setGoal(state.shoulderDegrees, state.elbowDegrees);
    }

    protected void setGoal(double shoulderDegrees, double elbowDegrees) {
    }
}