package com.stuypulse.robot.subsystems.arm;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmImpl extends Arm{
    private final TalonFX leftArmMotor;
    private final TalonFX rightArmMotor;
    private final TalonFX elbowMotor;
    
    private final Pigeon2 shoulderEncoder;
    private final DutyCycleEncoder elbowEncoder;

    public ArmImpl(){
        this.state = ArmState.L1;
        leftArmMotor = new TalonFX(Ports.Arm.LEFT_SHOULDER);
        rightArmMotor = new TalonFX(Ports.Arm.RIGHT_SHOULDER);
        elbowMotor = new TalonFX(Ports.Arm.ELBOW_MOTOR);
        shoulderEncoder = new Pigeon2(Ports.Arm.PIGEON);
        elbowEncoder = new DutyCycleEncoder(Ports.Arm.ABSOLUTE_ENCODER);

        //the left and right shoulder motors are moving in sync to my knowledge
        leftArmMotor.setControl(new Follower(Ports.Arm.RIGHT_SHOULDER, false));

        //configs when hardware is done

    }

    @Override
    public Rotation2d getShoulderAngle(){
        return shoulderEncoder.getRotation2d().minus(Settings.Arm.SHOULDER_ANGLE_OFFSET);

    }
    @Override
    public Rotation2d getElbowAngle(){
        return Rotation2d.fromRotations(elbowEncoder.get() - Settings.Arm.ELBOW_ANGLE_OFFSET.getRotations());
    }

    @Override
    public void periodic(){
        super.periodic();

        
        // later
    
}
}
