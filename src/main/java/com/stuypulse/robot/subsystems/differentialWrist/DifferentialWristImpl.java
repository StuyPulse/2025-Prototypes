package com.stuypulse.robot.subsystems.differentialWrist;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Motors.TalonFXConfig;

import edu.wpi.first.math.geometry.Rotation2d;

public class DifferentialWristImpl extends DifferentialWrist {
    private final TalonFX leftDifferentialMotor;
    private final TalonFX rightDifferentialMotor;
    private final TalonFX rollerMotor;

    public DifferentialWristImpl() {
        super();
        /*
         * Configure motors after Motors.java has been updated.
         */        
        leftDifferentialMotor = new TalonFX(Ports.DifferentialWrist.LEFT_DIFFERENTIAL, "*");
        TalonFXConfig leftConfig = new TalonFXConfig()
                .withCurrentLimitAmps(0)
                .withRampRate(0)
                .withNeutralMode(NeutralModeValue.Brake)
                .withInvertedValue(InvertedValue.CounterClockwise_Positive)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withSupplyCurrentLimitAmps(0)
                .withMotionProfile(0, 0)
                .withRemoteSensor(0, null, 0)
                .withSensorToMechanismRatio(0)
                .withFFConstants(0, 0, 0, 0,0)
                .withPIDConstants(0, 0, 0, 0);


        leftConfig.configure(leftDifferentialMotor);
            

        
        rightDifferentialMotor = new TalonFX(Ports.DifferentialWrist.RIGHT_DIFFERENTIAL, "*");
        TalonFXConfig rightConfig = new TalonFXConfig()
                .withCurrentLimitAmps(0)
                .withRampRate(0)
                .withNeutralMode(NeutralModeValue.Brake)
                .withInvertedValue(InvertedValue.CounterClockwise_Positive)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withSupplyCurrentLimitAmps(0)
                .withMotionProfile(0, 0)
                .withRemoteSensor(0, null, 0)
                .withSensorToMechanismRatio(0)
                .withFFConstants(0, 0, 0, 0,0)
                .withPIDConstants(0, 0, 0, 0);

        rightConfig.configure(leftDifferentialMotor);
        


        rollerMotor = new TalonFX(Ports.DifferentialWrist.ROLLER, "*");
        TalonFXConfig rollerConfig = new TalonFXConfig()
                .withCurrentLimitAmps(0)
                .withRampRate(0)
                .withNeutralMode(NeutralModeValue.Brake)
                .withInvertedValue(InvertedValue.CounterClockwise_Positive)
                .withSupplyCurrentLimitAmps(0)
                .withMotionProfile(0, 0)
                .withSensorToMechanismRatio(0);

        rollerConfig.configure(rollerMotor);


    }

    @Override
    public Rotation2d getCurrentRollAngle() {
        return Rotation2d.fromRotations((leftDifferentialMotor.getPosition().getValueAsDouble() - rightDifferentialMotor.getPosition().getValueAsDouble()) / 2);
    }

    @Override
    public Rotation2d getCurrentPitchAngle() {
        return Rotation2d.fromRotations((leftDifferentialMotor.getPosition().getValueAsDouble() + rightDifferentialMotor.getPosition().getValueAsDouble()) / 2);
    }

}
