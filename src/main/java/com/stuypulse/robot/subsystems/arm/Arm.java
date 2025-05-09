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
            instance = new ArmImpl();
        }
    }

    public static Arm getInstance(){
        return instance;
    }
    
    public abstract Rotation2d getShoulderAngle();
    public abstract Rotation2d getElbowAngle();


    public enum ArmState {
        L1(Rotation2d.fromDegrees(Settings.Arm.L1_SHOULDER), Rotation2d.fromDegrees(Settings.Arm.L1_ELBOW));


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

    protected ArmState state;

    public ArmState getState(){
        return this.state;
    }

    public void setState(ArmState state){
        this.state = state;
    }

    @Override
    public void periodic(){
        SmartDashboard.putString("Arm/State", state.toString());
    }
}