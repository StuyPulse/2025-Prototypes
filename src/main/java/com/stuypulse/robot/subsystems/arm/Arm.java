package com.stuypulse.robot.subsystems.arm;

import org.dyn4j.geometry.Rotation;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
        RESTING(Rotation2d.fromDegrees(Settings.Arm.RESTING_SHOULDER), Rotation2d.fromDegrees(Settings.Arm.RESTING_ELBOW));


        private Rotation2d shoulderTargetAngle;
        private Rotation2d elbowTargetAngle;

        private ArmState(Rotation2d shoulderTargetAngle, Rotation2d elbowTargetAngle){
            this.shoulderTargetAngle = Rotation2d.fromDegrees(
                SLMath.clamp(shoulderTargetAngle.getDegrees(), 
                Constants.Arm.MIN_ANGLE.getDegrees(), 
                Constants.Arm.MAX_ANGLE.getDegrees()));
            this.elbowTargetAngle = Rotation2d.fromDegrees(
                SLMath.clamp(elbowTargetAngle.getDegrees(), 
                Constants.Arm.MIN_ANGLE.getDegrees(), 
                Constants.Arm.MAX_ANGLE.getDegrees())); 
        } 

        public Rotation2d getShoulderTargetAngle(){
            return this.shoulderTargetAngle;
        }

        public Rotation2d getElbowTargetAngle(){
            return this.elbowTargetAngle;
        }
    }

    private ArmState state;

    public ArmState getState(){
        return this.state;
    }

    // Core methods
    public abstract Rotation2d getShoulderAngle();
    public abstract Rotation2d getElbowAngle();
    public abstract Translation2d getEndPosition();
    public abstract void setTargetAngles(Rotation2d shoulderAngle, Rotation2d elbowAngle);

    public void setState(ArmState state) {
        setGoal(state.getShoulderTargetAngle().getDegrees(), state.getShoulderTargetAngle().getDegrees());
    }

    protected void setGoal(double shoulderDegrees, double elbowDegrees) {
    }
}