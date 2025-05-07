package com.stuypulse.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;

public class ArmImpl extends Arm{
    private final TalonFX leftArmMotor;
    private final TalonFX rightArmMotor;
    private final TalonFX elbowMotor;

    public ArmImpl(){
        leftArmMotor = new TalonFX(Ports.Arm.LEFT_SHOULDER);
        rightArmMotor = new TalonFX(Ports.Arm.RIGHT_SHOULDER);
        elbowMotor = new TalonFX(Ports.Arm.ELBOW_MOTOR);

    }

    @Override
    public double getShoulderAngle(){
        return leftArmMotor.get();
    }

    @Override
    public double getElbowAngle(){
        return elbowMotor.get();
    }

}
