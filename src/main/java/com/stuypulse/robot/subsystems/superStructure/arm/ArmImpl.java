package com.stuypulse.robot.subsystems.superStructure.arm;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Settings;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.robot.constants.Ports;
import edu.wpi.first.math.geometry.Rotation2d;

import com.ctre.phoenix6.hardware.TalonFX;

public class ArmImpl extends Arm {
    private TalonFX left_motor;
    private TalonFX right_motor;
    //private ArmFeedforward FF;
    //private ProfiledPIDController PID;
       

    public ArmImpl() {
        left_motor = new TalonFX(Ports.Arm.LEFT_MOTOR);
        Motors.Arm.LEFT_MOTOR_CONFIG.configure(left_motor);
        left_motor.setPosition(Settings.Arm.MIN_ANGLE.getRotations());

        right_motor = new TalonFX(Ports.Arm.RIGHT_MOTOR);
        Motors.Arm.RIGHT_MOTOR_CONFIG.configure(right_motor);
        left_motor.setPosition(Settings.Arm.MIN_ANGLE.getRotations());

    }

    private boolean isWithinTolerance(Rotation2d tolerance) {
        return Math.abs(getCurrentAngle().getDegrees() - getTargetAngle().getDegrees()) < tolerance.getDegrees();
    }

    private Rotation2d getTargetAngle() {
        return Rotation2d.fromDegrees(SLMath.clamp(getState().getTargetAngle().getDegrees(),
                Settings.Arm.MIN_ANGLE.getDegrees(), Settings.Arm.MAX_ANGLE.getDegrees()));
    }

    @Override
    public boolean atTargetAngle() {
        return isWithinTolerance(Settings.Arm.ANGLE_TOLERANCE);
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(
                (left_motor.getPosition().getValueAsDouble() + right_motor.getPosition().getValueAsDouble()) / 2);
    }

    @Override
    public void periodic() {

        //left_motor.setVoltage(PID.calculate(getCurrentAngle().getDegrees(), getTargetAngle().getDegrees()) + 
        //FF.calculate(getTargetAngle().getDegrees(), 0));
        
        //right_motor.setVoltage(PID.calculate(getCurrentAngle().getDegrees(), getTargetAngle().getDegrees()) + 
        //FF.calculate(getTargetAngle().getDegrees(), 0));
        
        left_motor.setControl(
            new MotionMagicVoltage(getState().getTargetAngle().getRotations())
        );
        
        right_motor.setControl(
            new MotionMagicVoltage(getState().getTargetAngle().getRotations())
        );
        // smartdashboard

    }

}