package com.stuypulse.robot.subsystems.differentialWrist;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;

public class DifferentialWristImpl extends DifferentialWrist {
    private final TalonFX leftDifferentialMotor;
    private final TalonFX rightDifferentialMotor;
    private final TalonFX rollerMotor;
    
    private final CANcoder leftEncoder;
    private final CANcoder rightEncoder;
    
    public DifferentialWristImpl() {
        super();

        leftDifferentialMotor = new TalonFX(Ports.DifferentialWrist.LEFT_DIFFERENTIAL_MOTOR, "");
        Motors.DifferentialWrist.LEFT_DIFFERENTIAL_MOTOR_CONFIG.configure(leftDifferentialMotor);

        rightDifferentialMotor = new TalonFX(Ports.DifferentialWrist.RIGHT_DIFFERENTIAL_MOTOR, "");           
        Motors.DifferentialWrist.RIGHT_DIFFERENTIAL_MOTOR_CONFIG.configure(rightDifferentialMotor);

        rollerMotor = new TalonFX(Ports.DifferentialWrist.ROLLER_MOTOR, "");
        Motors.DifferentialWrist.ROLLER_MOTOR_CONFIG.configure(rollerMotor);

        leftEncoder = new CANcoder(Ports.DifferentialWrist.LEFT_ENCODER, "");
        rightEncoder = new CANcoder(Ports.DifferentialWrist.RIGHT_ENCODER, "");

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
     * The pitch (up and down) is given by the angle bisector of the current angles of the left and right motors.
     * The roll (left and right) is given by the distance / 2 between the current angles of the left and right motors.
     */
     
    @Override
    public Rotation2d getLeftCurrentAngle() {
        return Rotation2d.fromRotations(leftDifferentialMotor.getPosition().getValueAsDouble());
    }

    public Rotation2d getRightCurrentAngle() {
        return Rotation2d.fromRotations(rightDifferentialMotor.getPosition().getValueAsDouble());
    }
    
    @Override
    public void periodic() {
        super.periodic();

        rollerMotor.set(getRollerState().getTargetSpeed());
        
        leftDifferentialMotor.setControl(new MotionMagicVoltage(
           getLeftTargetAngle().getRotations()
        ));
        rightDifferentialMotor.setControl(new MotionMagicVoltage(
            getRightTargetAngle().getRotations()
        ));

       
    }

}
