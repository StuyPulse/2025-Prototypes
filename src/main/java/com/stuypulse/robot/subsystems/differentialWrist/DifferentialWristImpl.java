package com.stuypulse.robot.subsystems.differentialWrist;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.geometry.Rotation2d;

public class DifferentialWristImpl extends DifferentialWrist {
    private final SparkMax leftDifferentialMotor;
    private final SparkMax rightDifferentialMotor;

    private final AnglePIDController leftController;
    private final AnglePIDController rightController;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    
    
    
    public DifferentialWristImpl() {
        super();

        leftDifferentialMotor = new SparkMax(Ports.DifferentialWrist.LEFT_DIFFERENTIAL_MOTOR, MotorType.kBrushless);
        rightDifferentialMotor = new SparkMax(Ports.DifferentialWrist.RIGHT_DIFFERENTIAL_MOTOR,MotorType.kBrushless);

        leftDifferentialMotor.configure(Motors.DifferentialWrist.LEFT_DIFFERENTIAL_MOTOR_CONFIG, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        rightDifferentialMotor.configure(Motors.DifferentialWrist.RIGHT_DIFFERENTIAL_MOTOR_CONFIG, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);

        leftEncoder = leftDifferentialMotor.getEncoder();
        rightEncoder = leftDifferentialMotor.getEncoder();

        leftController = new AnglePIDController(0.5, 0, 0.1);
        rightController = new AnglePIDController(0.5, 0, 0.1);

        

        


        // leftDifferentialMotor.set(
        //     Rotation2d.fromRotations(leftDifferentialMotor.getEncoder().getPosition())
        //         .minus(Rotation2d.fromDegrees(Settings.DifferentialWrist.LEFT_ANGLE_OFFSET))
        //         .getRotations()
        // );

        // rightDifferentialMotor.set(
        //     Rotation2d.fromRotations(rightDifferentialMotor.getEncoder().getPosition())
        //         .minus(Rotation2d.fromDegrees(Settings.DifferentialWrist.RIGHT_ANGLE_OFFSET))
        //         .getRotations()
        // );
    }

    /**
     * Explanation of how differential motors work with 2 degrees of freedom:
     * The pitch (up and down) is given by the angle bisector of the current angles of the left and right motors.
     * The roll (left and right) is given by the distance / 2 between the current angles of the left and right motors.
     */
     
    @Override
    public Rotation2d getLeftCurrentAngle() {
        return Rotation2d.fromRotations(leftEncoder.getPosition());
    }

    public Rotation2d getRightCurrentAngle() {  
        return Rotation2d.fromRotations(rightEncoder.getPosition());
    }
    
    @Override
    public void periodic() {
        super.periodic();

        leftController.update(
            Angle.fromRotations(getLeftTargetAngle().getRotations()),
            Angle.fromRotations(getLeftCurrentAngle().getRotations())
        );

        rightController.update(
            Angle.fromRotations(getRightTargetAngle().getRotations()),
            Angle.fromRotations(getRightCurrentAngle().getRotations())
        );


        double leftVoltage = leftController.getOutput();
        double rightVoltage = rightController.getOutput();

        leftDifferentialMotor.setVoltage(leftVoltage);
        rightDifferentialMotor.setVoltage(rightVoltage);


        

    }

}
