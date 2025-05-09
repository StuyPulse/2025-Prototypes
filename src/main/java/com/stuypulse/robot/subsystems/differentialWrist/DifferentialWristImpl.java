package com.stuypulse.robot.subsystems.differentialWrist;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DifferentialWristImpl extends DifferentialWrist {
    private final TalonFX leftDifferentialMotor;
    private final TalonFX rightDifferentialMotor;
    private final TalonFX rollerMotor;
    
    private final CANcoder leftEncoder;
    private final CANcoder rightEncoder;
    
    public DifferentialWristImpl() {
        super();

        leftDifferentialMotor = new TalonFX(Ports.DifferentialWrist.LEFT_DIFFERENTIAL_MOTOR, "*");
        Motors.DifferentialWrist.LEFT_DIFFERENTIAL_MOTOR_CONFIG.configure(leftDifferentialMotor);

        rightDifferentialMotor = new TalonFX(Ports.DifferentialWrist.RIGHT_DIFFERENTIAL_MOTOR, "*");           
        Motors.DifferentialWrist.RIGHT_DIFFERENTIAL_MOTOR_CONFIG.configure(rightDifferentialMotor);

        rollerMotor = new TalonFX(Ports.DifferentialWrist.ROLLER_MOTOR, "*");
        Motors.DifferentialWrist.ROLLER_MOTOR_CONFIG.configure(rollerMotor);

        leftEncoder = new CANcoder(Ports.DifferentialWrist.LEFT_ENCODER, "*");
        rightEncoder = new CANcoder(Ports.DifferentialWrist.RIGHT_ENCODER, "*");

        leftDifferentialMotor.setPosition(
            Rotation2d.fromRotations(leftEncoder.getPosition().getValueAsDouble())
                .minus(Rotation2d.fromDegrees(Settings.DifferentialWrist.LEFT_ANGLE_OFFSET))
                .getRotations()
        );

        rightDifferentialMotor.setPosition(
            Rotation2d.fromRotations(rightEncoder.getPosition().getValueAsDouble())
                .minus(Rotation2d.fromDegrees(Settings.DifferentialWrist.RIGHT_ANGLE_OFFSET))
                .getRotations()
        );
    }

    /**
     * Explanation of how differential motors work with 2 degrees of freedom:
     * The pitch (up and down) is given by the midpoint of the current angles of the left and right motors.
     * The roll (left and right) is given by the distance / 2 between the current angles of the left and right motors.
     */

    public Rotation2d getLeftCurrentAngle() {
        return Rotation2d.fromRotations(leftDifferentialMotor.getPosition().getValueAsDouble());
    }

    public Rotation2d getRightCurrentAngle() {
        return Rotation2d.fromRotations(rightDifferentialMotor.getPosition().getValueAsDouble());
    }

    @Override
    public Rotation2d getCurrentRollAngle() {
        return Rotation2d.fromRotations((getLeftCurrentAngle().getRotations() - getRightCurrentAngle().getRotations()) / 2);
    }

    @Override
    public Rotation2d getCurrentPitchAngle() {
        return Rotation2d.fromRotations((getLeftCurrentAngle().getRotations() + getRightCurrentAngle().getRotations()) / 2);
    }

    @Override
    public boolean isAtTargetPitchAngle() {
        return Math.abs(getTargetPitchAngle().getDegrees() - getCurrentPitchAngle().getDegrees()) < Settings.DifferentialWrist.PITCH_ANGLE_TOLERANCE;
    }

    @Override
    public boolean isAtTargetRollAngle() {
        return Math.abs(getTargetRollAngle().getDegrees() - getCurrentRollAngle().getDegrees()) < Settings.DifferentialWrist.ROLL_ANGLE_TOLERANCE;
    }

    private Rotation2d getTargetPitchAngle() {
        return Rotation2d.fromDegrees(
            SLMath.clamp(
                getWristState().getPitch().getDegrees(),
                Settings.DifferentialWrist.MIN_PITCH_ANGLE,
                Settings.DifferentialWrist.MAX_PITCH_ANGLE));
    }

    private Rotation2d getTargetRollAngle() {
        return Rotation2d.fromDegrees(
            SLMath.clamp(
                getWristState().getRoll().getDegrees(),
                Settings.DifferentialWrist.MIN_ROLL_ANGLE,
                Settings.DifferentialWrist.MAX_ROLL_ANGLE));
    }

    @Override
    public void periodic() {
        super.periodic();

        rollerMotor.set(getRollerState().getTargetSpeed());
        
        leftDifferentialMotor.setControl(new MotionMagicVoltage(
            getLeftCurrentAngle().getRotations() +
            (getTargetPitchAngle().getRotations() - getCurrentPitchAngle().getRotations()) +
            (getTargetRollAngle().getRotations() - getCurrentRollAngle().getRotations())
        ));
        rightDifferentialMotor.setControl(new MotionMagicVoltage(
            getRightCurrentAngle().getRotations() +
            (getTargetPitchAngle().getRotations() - getCurrentPitchAngle().getRotations()) -
            (getTargetRollAngle().getRotations() - getCurrentRollAngle().getRotations())
        ));

        SmartDashboard.putNumber("Differential Wrist/Wrist/Left Motor Angle (deg)", getLeftCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Differential Wrist/Wrist/Right Motor Angle (deg)", getRightCurrentAngle().getDegrees());

        SmartDashboard.putBoolean("Differential Wrist/Wrist/At Target Pitch Angle", isAtTargetPitchAngle());
        SmartDashboard.putBoolean("Differential Wrist/Wrist/At Target Roll Angle", isAtTargetRollAngle());

        SmartDashboard.putNumber("Differential Wrist/Wrist/Current Pitch Angle (deg)", getCurrentPitchAngle().getDegrees());
        SmartDashboard.putNumber("Differential Wrist/Wrist/Current Roll Angle (deg)", getCurrentRollAngle().getDegrees());
        SmartDashboard.putNumber("Differential Wrist/Wrist/Target Pitch Angle (deg)", getTargetPitchAngle().getDegrees());
        SmartDashboard.putNumber("Differential Wrist/Wrist/Target Roll Angle (deg)", getTargetRollAngle().getDegrees());
    }

}
