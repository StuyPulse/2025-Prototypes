package com.stuypulse.robot.subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
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

public class ElevatorImpl extends SubsystemBase {

    private static final ElevatorImpl instance;

    static {
        instance = new ElevatorImpl();
    }

    public static ElevatorImpl getInstance() {
        return instance;
    }

    private final SmartNumber targetHeight;
    private ElevatorFeedforward FF;
    private ProfiledPIDController PID;

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final SparkMaxConfig leftConfig;
    private final SparkMaxConfig rightConfig;

    private final ElevatorVisualizer visualizer;

    public ElevatorImpl() {
        targetHeight = new SmartNumber("Elevator/Target Height", 0);

        leftMotor = new SparkMax(Ports.Elevator.LEFT, MotorType.kBrushless);
        rightMotor = new SparkMax(Ports.Elevator.RIGHT, MotorType.kBrushless);

        rightConfig = new SparkMaxConfig();
        leftConfig = new SparkMaxConfig();

        rightConfig.encoder.positionConversionFactor(Settings.Elevator.POSITION_CONVERSION_FACTOR);
        leftConfig.encoder.positionConversionFactor(Settings.Elevator.POSITION_CONVERSION_FACTOR);
        rightConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);
        leftConfig.idleMode(SparkMaxConfig.IdleMode.kBrake);

        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        rightMotor.configure(leftConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        leftMotor.configure(rightConfig, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters);
        
        visualizer = new ElevatorVisualizer(this);

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
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
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

        visualizer.update();
    }

}
    