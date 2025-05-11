package com.stuypulse.robot.subsystems.superStructure.arm;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase {

private static final Arm instance;

  static {
        if (Robot.isReal()) {
            instance = new ArmImpl();
        }
        // else {
        //     instance = new ArmSim();
        // }
    }

    public static Arm getInstance() {
        return instance;
    }

    public enum ArmState {
        S0(Settings.Arm.MIN_ANGLE),
        L1(Settings.Arm.L1_ANGLE),
        L2_FRONT(Settings.Arm.L2_ANGLE_FRONT),
        L3_FRONT(Settings.Arm.L3_ANGLE_FRONT),
        L4_FRONT(Settings.Arm.L4_ANGLE_FRONT),
        L2_BACK(Settings.Arm.L2_ANGLE_BACK),
        L3_BACK(Settings.Arm.L3_ANGLE_BACK),
        L4_BACK(Settings.Arm.L4_ANGLE_BACK);
        
        private Rotation2d targetAngle;

        private ArmState(Rotation2d targetAngle) {
            this.targetAngle = Rotation2d.fromDegrees(SLMath.clamp(targetAngle.getDegrees(), Settings.Arm.MIN_ANGLE.getDegrees(), Settings.Arm.MAX_ANGLE.getDegrees()));
        }

        public Rotation2d getTargetAngle() {
            return this.targetAngle;
        }

    }

    private ArmState state;


    public ArmState getState() {
        return this.state;
    }

    public void setState(ArmState state) {
        this.state = state;
    }

    public abstract Rotation2d getCurrentAngle();
    public abstract boolean atTargetAngle();
    
    @Override
    public void periodic() {
        //SmartDashboard stuff 
    }
}

    