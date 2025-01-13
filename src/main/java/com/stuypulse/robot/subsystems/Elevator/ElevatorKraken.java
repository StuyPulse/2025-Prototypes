package com.stuypulse.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Elevator.Feedforward;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
2 motors
4 rollers
*/

public class ElevatorKraken extends SubsystemBase {

    private static final ElevatorKraken instance;

    static {
        instance = new ElevatorKraken();
    }

    public static ElevatorKraken getInstance() {
        return instance;
    }

    private final SmartNumber targetHeight;
    private ElevatorFeedforward FF;
    private ProfiledPIDController PID;

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;


    public ElevatorKraken() {
        targetHeight = new SmartNumber("Elevator/Target Height", 0);

        leftMotor = new TalonFX(Ports.Elevator.LEFT);
        rightMotor = new TalonFX(Ports.Elevator.RIGHT);

        
        FF = new ElevatorFeedforward(
            Feedforward.kS, 
            Feedforward.kG, 
            Feedforward.kV
        );

        PID = new ProfiledPIDController(
            Settings.Elevator.PID.kP, 
            Settings.Elevator.PID.kI, 
            Settings.Elevator.PID.kD, 
            new TrapezoidProfile.Constraints(
                Settings.Elevator.MAX_ACCELERATION, 
                Settings.Elevator.MAX_VELOCITY
            )
        );

    }

    public void setTargetHeight(double height) {
        targetHeight.set(
            SLMath.clamp(
                height, 
                Settings.Elevator.MIN_HEIGHT, 
                Settings.Elevator.MAX_HEIGHT
            )
        );
    }

    public double getTargetHeight() {
        return targetHeight.getAsDouble();
    }


    public double getHeight() {
        return 
            ((leftMotor.getPosition().getValueAsDouble() * Settings.Elevator.POSITION_CONVERSION_FACTOR) + 
            (rightMotor.getPosition().getValueAsDouble() * Settings.Elevator.POSITION_CONVERSION_FACTOR)) / 2.0;
    }

    public void stopElevator() {
        rightMotor.stopMotor();
        leftMotor.stopMotor(); 
    }
    
    public void periodic() {

        final double PIDOutput = PID.calculate(getHeight(), targetHeight.doubleValue());
        final double FFOutput = FF.calculate(PID.getSetpoint().velocity);

        leftMotor.setVoltage(PIDOutput + FFOutput);
        rightMotor.setVoltage(PIDOutput + FFOutput);
        
        SmartDashboard.putNumber("Elevator/Target Height", targetHeight.getAsDouble());
        SmartDashboard.putNumber("Elevator/Height", getHeight());

    }

}   
    