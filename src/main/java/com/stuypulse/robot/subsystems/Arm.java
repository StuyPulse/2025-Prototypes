package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.RobotType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Arm extends SubsystemBase{
    

    private static final Arm instance;
    
    public static enum State {
        REST(0.0),
        L1(Settings.Arm.L1_ANGLE),
        L2(Settings.Arm.L2_ANGLE),
        L3(Settings.Arm.L3_ANGLE),
        L4(Settings.Arm.L4_ANGLE);

        private double angle;
        private State(double angle) {
            this.angle = angle;
        }

        private double getStateAngle() {
            return angle;
        }

        private double getStateAngleRadians() {
            return Units.degreesToRadians(angle);
        }
    }

    private State state;

    static {
        instance = new Arm();

    }

    public static Arm getInstance() {
        return instance;
    }

    private static SparkMax motor;

    private static RelativeEncoder relativeEncoder;

    private final ProfiledPIDController controller;

    private final ArmFeedforward ffController;


    public Arm() {
        state = State.REST;
        motor = new SparkMax(Ports.Arm.MOTOR, MotorType.kBrushless);
        relativeEncoder = motor.getAlternateEncoder();
        controller = new ProfiledPIDController(Settings.Arm.PID.kP, Settings.Arm.PID.kI, Settings.Arm.PID.kD, new Constraints(Settings.Arm.MAX_VEL, Settings.Arm.MAX_ACCEL));
        ffController = new ArmFeedforward(Settings.Arm.FF.kS, Settings.Arm.FF.kG, Settings.Arm.FF.kV, Settings.Arm.FF.kA);
        
        controller.enableContinuousInput(-180, 180);

    }

    public double getAngle() {
        return Units.rotationsToDegrees(relativeEncoder.getPosition());
    }

    public void setState(State state) {
        this.state = state;
    }

    public State getState() {
        return state;
    }

    public final boolean isAtTargetState(double epsilonDegrees) {
        return Math.abs(getState().getStateAngle() - getAngle()) < epsilonDegrees;
    }

    public void reset(double angle) {  // NOT FINISHED
        relativeEncoder.setPosition(angle);
        setState(State.REST);
    }
    @Override
    public void periodic() {
        motor.setVoltage(controller.calculate(
            getAngle(), getState().getStateAngle()) 
        + ffController.calculate(getState().getStateAngleRadians(), 0)
        );
    }
}
