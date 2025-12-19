package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.util.FieldUtil;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;

import java.util.function.Supplier;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwervePIDToPose extends Command {
    private SwerveDrive swerve;
    private Odometry odometry;
    private Supplier<Pose2d> targetPose;
    private Pose2d robotPose;

    private PIDController xController;
    private PIDController yController;
    private PIDController thetaController;
    private Gamepad controller;
    private Pose2d usedPose;

    private final FieldObject2d targetPose2d;

    public SwervePIDToPose(Pose2d targetPose, Gamepad controller) {
        this(() -> targetPose, controller);
    }

    public SwervePIDToPose(Supplier<Pose2d> targetPose, Gamepad controller) {
        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();
        this.targetPose = targetPose;

        targetPose2d = odometry.getField().getObject("Swerve Target Pose");

        // PID Controller setups
        xController = new PIDController(Gains.pidToPose.x.kP, Gains.pidToPose.x.kI, Gains.pidToPose.x.kD);
        yController = new PIDController(Gains.pidToPose.y.kP, Gains.pidToPose.y.kI, Gains.pidToPose.y.kD);
        thetaController = new PIDController(Gains.pidToPose.theta.kP, Gains.pidToPose.theta.kI,
                Gains.pidToPose.theta.kD);

        this.controller = controller;

        addRequirements(swerve, odometry);
    }

    boolean isAlignedX() {
        return Math.abs(targetPose.get().getX() - robotPose.getX()) < Settings.Swerve.Alignment.X_TOLERANCE
                .getAsDouble();
    }

    boolean isAlignedY() {
        return Math.abs(targetPose.get().getY() - robotPose.getY()) < Settings.Swerve.Alignment.Y_TOLERANCE
                .getAsDouble();
    }

    boolean isAlignedTheta() {
        return Math.abs(targetPose.get().getRotation().minus(robotPose.getRotation())
                .getRadians()) < Settings.Swerve.Alignment.THETA_TOLERANCE.getAsDouble();
    }

    @Override 
    public void initialize() {
        usedPose = targetPose.get();
    }
    
    @Override
    public void execute() {
        robotPose = odometry.getPose();
        targetPose2d.setPose(usedPose);

        // Pid outputs
        double outX = xController.calculate(robotPose.getX(), usedPose.getX());
        double outY = yController.calculate(robotPose.getY(), usedPose.getY());
        double outTheta = thetaController.calculate(robotPose.getRotation().getRadians(),
        usedPose.getRotation().getRadians());

        ChassisSpeeds swerveChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                outX,
                outY,
                outTheta,
                robotPose.getRotation());

        swerve.setChassisSpeeds(swerveChassisSpeeds);

        SmartDashboard.putNumber("Alignment/Target x", usedPose.getX());
        SmartDashboard.putNumber("Alignment/Target y", usedPose.getY());
        SmartDashboard.putNumber("Alignment/Target angle", usedPose.getRotation().getDegrees());

        SmartDashboard.putBoolean("Alignment/Is Aligned X", isAlignedX());
        SmartDashboard.putBoolean("Alignment/Is Aligned Y", isAlignedY());
        SmartDashboard.putBoolean("Alignment/Is Aligned Theta", isAlignedTheta());

        SmartDashboard.putNumber("Alignment/Target Velocity Robot Relative X (m per s)", outX);
        SmartDashboard.putNumber("Alignment/Target Velocity Robot Relative Y (m per s)", outY);
        SmartDashboard.putNumber("Alignment/Target Angular Velocity (rad per s)", outTheta);

    }

    @Override
    public boolean isFinished() {
        // Finished if controller moves or robot aligns
        return (isAlignedX() && isAlignedY() && isAlignedTheta())
                ||
                stickmoved().get();
    }

    private Supplier<Boolean> stickmoved() {
        return () -> ((new Translation2d(controller.getLeftX(), controller.getLeftY()).getNorm() > 0.5)
                || (new Translation2d(controller.getRightX(), controller.getRightY()).getNorm() > 0.5));
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setChassisSpeeds(
                ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, Rotation2d.kZero));
        SmartDashboard.putBoolean("Alignment/is finished?", true);
        Field.clearFieldObject(targetPose2d);
    }
}
