package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartNumber;
import com.revrobotics.AbsoluteEncoder;


import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class Arm extends SubsystemBase{

    private static final Arm instance;

    static {
        instance = new Arm();
    }

    public static Arm getInstance() {
        return instance;
    }

    /*** Arm State ***/

    private SmartNumber targetAngle;

    /*** Hardware ***/

    private final SparkMax armMotor;
    private final AbsoluteEncoder armEncoder;

    /*** Controllers ***/

    private final ProfiledPIDController pidController;
    private final ArmFeedforward ffController;


    /*** Visualization ***/
    
    
    // private final ArmVisualizer visualizer;

    /*** Constructor ***/

    public Arm() {
        // Arm State
        targetAngle = new SmartNumber("Arm/Target Angle", 0.0);

        // Hardware
        armMotor = new SparkMax(Ports.Arm.MOTOR, MotorType.kBrushless);
        armEncoder = armMotor.getAbsoluteEncoder();

        // Controllers
        pidController = new ProfiledPIDController(
            Settings.Arm.PID.kP.getAsDouble(), 
            Settings.Arm.PID.kI.getAsDouble(), 
            Settings.Arm.PID.kD.getAsDouble(), 
            new Constraints(
                Settings.Arm.MAX_VEL, 
                Settings.Arm.MAX_ACCEL
            )
        );
        
        pidController.enableContinuousInput(-180, 180);
        
        ffController = new ArmFeedforward(
            Settings.Arm.FF.kS.getAsDouble(), 
            Settings.Arm.FF.kG, 
            Settings.Arm.FF.kV.getAsDouble(), 
            Settings.Arm.FF.kA.getAsDouble()
        );
        

        // Visualization

        

        // visualizer = new ArmVisualizer();
    }

    /// State

    public void setTargeAngle(double angle) {
        targetAngle.set(angle);
    }

    public double getTargetAngle() {
        return targetAngle.get();
    }
    
    /// Hardware
    public double getArmAngle() {
        return Units.rotationsToDegrees(armEncoder.getPosition());
    }
    
    
    @Override
    public void periodic() {
        
        armMotor.setVoltage(
            pidController.calculate(
                getArmAngle(), 
                getTargetAngle()
            ) +
            ffController.calculate(
                getTargetAngle(), 
                0
            )
        );

        ArmVisualizer.getInstance().update();
    }
}
