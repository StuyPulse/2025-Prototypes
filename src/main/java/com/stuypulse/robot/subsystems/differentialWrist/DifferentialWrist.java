package com.stuypulse.robot.subsystems.differentialWrist;

import com.stuypulse.robot.Robot;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DifferentialWrist extends SubsystemBase {
    public static final DifferentialWrist instance;

    static {
        if(Robot.isReal()) {
            instance = new DifferentialWristImpl();
        } else {
            instance = new DifferentialWristSim();
        }
    }

    public static DifferentialWrist getInstance() {
        return instance;
    }

    public enum WristState {
        STOW(new Rotation2d(),new Rotation2d()),
        CORAL_SCORE_L1(new Rotation2d(),new Rotation2d()),
        CORAL_SCORE_L2(new Rotation2d(),new Rotation2d()),
        CORAL_SCORE_L3(new Rotation2d(),new Rotation2d()),
        CORAL_SCORE_L4(new Rotation2d(),new Rotation2d()),
        ALGAE_SCORE_BARGE(new Rotation2d(),new Rotation2d()),
        ALGAE_SCORE_PROCESSOR(new Rotation2d(),new Rotation2d());

        private Rotation2d pitch, roll;

        private WristState(Rotation2d pitch, Rotation2d roll) {
            this.pitch = Rotation2d.fromDegrees(
                SLMath.clamp(pitch.getDegrees(), 0, 0)
            );
            this.roll = Rotation2d.fromDegrees(
                SLMath.clamp(roll.getDegrees(),0,0)
            );
        }

        public Rotation2d getPitch() {
            return this.pitch;
        }

        public Rotation2d getRoll() {
            return this.roll;
        }
    }

    public enum RollerState {
        INTAKE_CORAL(0),
        INTAKE_ALGAE(0),
        SHOOT_CORAL(0),
        SHOOT_ALGAE(0),
        HOLD_ALGAE(0),
        HOLD_CORAL(0),
        STOP(0);

        private double speed;

        private RollerState(double speed) {
            this.speed = speed;
        }

        public double getTargetSpeed() {
            return speed;
        }
    }

    private WristState wristState;
    private RollerState rollerState;

    public DifferentialWrist(){
        this.wristState = WristState.STOW;
        this.rollerState = RollerState.STOP;
    }

    public WristState getWristState() {
        return wristState;
    }

    public void setWristState(WristState state) {
        this.wristState = state;
    }

    public RollerState getRollerState() {
        return rollerState;
    }

    public void setRollerState(RollerState state) {
        this.rollerState = state;
    }

    public abstract Rotation2d getCurrentPitchAngle();

    public abstract Rotation2d getCurrentRollAngle();

    public abstract boolean isAtTargetPitchAngle();

    public abstract boolean isAtTargetRollAngle();

    @Override
    public void periodic() {
        SmartDashboard.putString("Differntial Wrist/Wrist State", getWristState().toString());
        SmartDashboard.putString("Differential Wrist/Roller State", getRollerState().toString());
    }
}
