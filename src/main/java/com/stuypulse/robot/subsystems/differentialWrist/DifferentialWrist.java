package com.stuypulse.robot.subsystems.differentialWrist;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class DifferentialWrist extends SubsystemBase {
    public static final DifferentialWrist instance;
    public SmartNumber rollTargetAngle;
    public SmartNumber pitchTargetAngle;

    public Rotation2d leftTargetAngle;
    public Rotation2d rightTargetAngle;

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
        STOW(Rotation2d.fromDegrees(Settings.DifferentialWrist.STOW_PITCH_ANGLE), Rotation2d.fromDegrees(Settings.DifferentialWrist.STOW_ROLL_ANGLE)),
        CORAL_SCORE_L1(Rotation2d.fromDegrees(Settings.DifferentialWrist.CORAL_SCORE_L1_PITCH_ANGLE), Rotation2d.fromDegrees(Settings.DifferentialWrist.CORAL_SCORE_L1_ROLL_ANGLE)),
        CORAL_SCORE_L2(Rotation2d.fromDegrees(Settings.DifferentialWrist.CORAL_SCORE_L2_PITCH_ANGLE), Rotation2d.fromDegrees(Settings.DifferentialWrist.CORAL_SCORE_L2_ROLL_ANGLE)),
        CORAL_SCORE_L3(Rotation2d.fromDegrees(Settings.DifferentialWrist.CORAL_SCORE_L3_PITCH_ANGLE), Rotation2d.fromDegrees(Settings.DifferentialWrist.CORAL_SCORE_L3_ROLL_ANGLE)),
        CORAL_SCORE_L4(Rotation2d.fromDegrees(Settings.DifferentialWrist.CORAL_SCORE_L4_PITCH_ANGLE), Rotation2d.fromDegrees(Settings.DifferentialWrist.CORAL_SCORE_L4_ROLL_ANGLE)),
        ALGAE_SCORE_BARGE(Rotation2d.fromDegrees(Settings.DifferentialWrist.ALGAE_SCORE_BARGE_PITCH_ANGLE), Rotation2d.fromDegrees(Settings.DifferentialWrist.ALGAE_SCORE_BARGE_ROLL_ANGLE)),
        ALGAE_SCORE_PROCESSOR(Rotation2d.fromDegrees(Settings.DifferentialWrist.ALGAE_SCORE_PROCESSOR_PITCH_ANGLE), Rotation2d.fromDegrees(Settings.DifferentialWrist.ALGAE_SCORE_PROCESSOR_ROLL_ANGLE));

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

    private DifferentialWristVisualizer visualizer;


    public DifferentialWrist(){
        this.wristState = WristState.STOW;
        this.rollerState = RollerState.STOP;

        this.rollTargetAngle = new SmartNumber(
            "Differential Wrist/Roll Target",
            SLMath.clamp(
                getWristState().getPitch().getDegrees(),
                Settings.DifferentialWrist.MIN_PITCH_ANGLE,
                Settings.DifferentialWrist.MAX_PITCH_ANGLE)
        );
        this.pitchTargetAngle = new SmartNumber(
            "Differential Wrist/Pitch Target",
            SLMath.clamp(
                getWristState().getRoll().getDegrees(),
                Settings.DifferentialWrist.MIN_PITCH_ANGLE,
                Settings.DifferentialWrist.MAX_PITCH_ANGLE)
        );

        visualizer = DifferentialWristVisualizer.getInstance();
    }

    public WristState getWristState() {
        return wristState;
    }

    public void setWristState(WristState state) {
        this.wristState = state;
        this.rollTargetAngle.set(state.roll.getDegrees());
        this.pitchTargetAngle.set(state.pitch.getDegrees());

    }

    public RollerState getRollerState() {
        return rollerState;
    }

    public void setRollerState(RollerState state) {
        this.rollerState = state;
    }

    public Rotation2d getTargetPitchAngle() { 
        return Rotation2d.fromDegrees(pitchTargetAngle.get());
    }

    public Rotation2d getTargetRollAngle() {
        return Rotation2d.fromDegrees(rollTargetAngle.get());
    }

    public Rotation2d getLeftTargetAngle() {
        return leftTargetAngle;
    }

    public Rotation2d getRightTargetAngle() {
        return rightTargetAngle;
    }

    public boolean isAtTargetPitchAngle() {
        return Math.abs(getTargetPitchAngle().getDegrees() - getCurrentPitchAngle().getDegrees()) < Settings.DifferentialWrist.PITCH_ANGLE_TOLERANCE;
    }

    public boolean isAtTargetRollAngle() {
        return Math.abs(getTargetRollAngle().getDegrees() - getCurrentRollAngle().getDegrees()) < Settings.DifferentialWrist.ROLL_ANGLE_TOLERANCE;
    }

    public Rotation2d getCurrentRollAngle() {
        return Rotation2d.fromRotations((getLeftCurrentAngle().getRotations() - getRightCurrentAngle().getRotations()) / 2);
    }

    public Rotation2d getCurrentPitchAngle() {
        return Rotation2d.fromRotations((getLeftCurrentAngle().getRotations() + getRightCurrentAngle().getRotations()) / 2);
    }

    public abstract Rotation2d getLeftCurrentAngle();

    public abstract Rotation2d getRightCurrentAngle();

    

    @Override
    public void periodic() {
        visualizer.updateVisualizer();

        leftTargetAngle = Rotation2d.fromRotations(getLeftCurrentAngle().getRotations() +
        (getTargetPitchAngle().getRotations() - getCurrentPitchAngle().getRotations()) +
        (getTargetRollAngle().getRotations() - getCurrentRollAngle().getRotations()));

        rightTargetAngle = Rotation2d.fromRotations(getRightCurrentAngle().getRotations() +
        (getTargetPitchAngle().getRotations() - getCurrentPitchAngle().getRotations()) -
        (getTargetRollAngle().getRotations() - getCurrentRollAngle().getRotations()));

        SmartDashboard.putString("Differential Wrist/Wrist State", getWristState().toString());
        SmartDashboard.putString("Differential Wrist/Roller State", getRollerState().toString());

        SmartDashboard.putNumber("Differential Wrist/Left Motor Angle (deg)", getLeftCurrentAngle().getDegrees() % 360);
        SmartDashboard.putNumber("Differential Wrist/Right Motor Angle (deg)", getRightCurrentAngle().getDegrees() % 360);

        SmartDashboard.putBoolean("Differential Wrist/At Target Pitch Angle", isAtTargetPitchAngle());
        SmartDashboard.putBoolean("Differential Wrist/At Target Roll Angle", isAtTargetRollAngle());

        SmartDashboard.putNumber("Differential Wrist/Current Pitch Angle (deg)", getCurrentPitchAngle().getDegrees() % 360);
        SmartDashboard.putNumber("Differential Wrist/Current Roll Angle (deg)", getCurrentRollAngle().getDegrees() % 360);
    }
}
