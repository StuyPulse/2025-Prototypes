package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.RobotType;
import com.stuypulse.stuylib.network.SmartNumber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;

public class VFourBar extends SubsystemBase{

    private static final VFourBar instance;

    static {
        if (Robot.ROBOT == RobotType.AUNT_MARY) {
            instance = new VFourBar();
        } else {
            instance = new SimVFourBar();
        }
    }

    public static VFourBar getInstance() {
        return instance;
    }

    private static SparkMax motor;

    private static RelativeEncoder relativeEncoder;

    private final PIDController controller;

    private SmartNumber targetAngle;


    public VFourBar() {
        motor = new SparkMax(Ports.VFourBar.MOTOR, MotorType.kBrushless);
        relativeEncoder = motor.getAlternateEncoder();
        controller = new PIDController(Settings.VFourBar.PID.KP, Settings.VFourBar.PID.KI, Settings.VFourBar.PID.KD);
        targetAngle = new SmartNumber("VFourBar/Target Angle", 0.0);
        reset(Settings.VFourBar.LOWER_ANGLE_LIMIT);
    }

    public double getAngle() {
        return Rotation2d.fromRotations(relativeEncoder.getPosition()).getRadians();
    }

    public void setAngle(double angle) {
        targetAngle.set(MathUtil.clamp(angle, Settings.VFourBar.LOWER_ANGLE_LIMIT, Settings.VFourBar.UPPER_ANGLE_LIMIT));
    }

    // public void setL1() {
    //     setAngle(Settings.VFourBar.L1_ANGLE);
    // }

    // public void setL2() {
    //     setAngle(Settings.VFourBar.L2_ANGLE);
    // }

    // public void setL3() {
    //     setAngle(Settings.VFourBar.L3_ANGLE);
    // }

    // public void setL4() {
    //     setAngle(Settings.VFourBar.L4_ANGLE);
    // }

    public void reset(double angle) { // NOT FINISHED
        relativeEncoder.setPosition(angle);
        targetAngle.set(angle);
    }

    @Override
    public void periodic() {
        motor.setVoltage(controller.calculate(getAngle(), targetAngle.doubleValue()*Settings.VFourBar.DEG_TO_RAD));
    }

}
