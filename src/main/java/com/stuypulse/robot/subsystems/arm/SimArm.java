package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;

public class SimArm extends SubsystemBase{
    public static SimArm instance = new SimArm();
    private final SingleJointedArmSim sim;

    private final ProfiledPIDController pidController;
    private final ArmFeedforward ffController;

    private final SmartNumber targetAngle;

    // simulation 

    private final Mechanism2d stick;
    private final MechanismRoot2d link;
    private final MechanismLigament2d stickStick;
    private final MechanismLigament2d slowStick;

    protected SimArm() {
        sim = new SingleJointedArmSim(
                DCMotor.getKrakenX60(1),
                Settings.Arm.GEAR_RATIO,
                Settings.Arm.MOMENT_OF_INERTIA,
                Settings.Arm.ARM_LENGTH,
                Settings.Arm.LOWER_ANGLE_LIMIT,
                Settings.Arm.UPPER_ANGLE_LIMIT,
                false,
                Settings.Arm.LOWER_ANGLE_LIMIT,
                0,0);



        pidController = new ProfiledPIDController(
            Settings.Arm.PID.kP.getAsDouble(), 
            Settings.Arm.PID.kI.getAsDouble(), 
            Settings.Arm.PID.kD.getAsDouble(), 
            new Constraints(
                Settings.Arm.MAX_VEL, 
                Settings.Arm.MAX_ACCEL
            )
        );

        ffController = new ArmFeedforward(
            Settings.Arm.FF.kS.getAsDouble(), 
            Settings.Arm.FF.kG, 
            Settings.Arm.FF.kV.getAsDouble(), 
            Settings.Arm.FF.kA.getAsDouble()
        );

        targetAngle = new SmartNumber("SimArm/Target Angle", 0.0);

        // Simulation 

        stick = new Mechanism2d(100, 100);

        stickStick = new MechanismLigament2d(
        "arm",
        30,
        0,
        10,
        new Color8Bit(Color.kAqua));

        slowStick = new MechanismLigament2d(
            "slow arm",
            20,
            0,
            10,
            new Color8Bit(Color.kLimeGreen)
            
        );

        link = stick.getRoot("Root Origin", 30, 30);
        link.append(stickStick);
        link.append(slowStick);

    }

    public static SimArm getInstance() {
        return instance;
    }

    public double getTargetAngle() {
        return targetAngle.getAsDouble();
    }

    public double getArmAngle() {
        return Rotation2d.fromRadians(sim.getAngleRads()).getDegrees();
    }

    @Override
    public void periodic() {
        
        sim.setInputVoltage(
            pidController.calculate(
                getArmAngle(), 
                getTargetAngle()
            ) +
            ffController.calculate(
                getTargetAngle(), 
                0
            )
        );
        sim.update(0.020);


        SmartDashboard.putData("SimArm", stick);
        stickStick.setAngle(getTargetAngle());
        slowStick.setAngle(getArmAngle());
    }


    
}
