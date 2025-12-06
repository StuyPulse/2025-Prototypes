package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SwervePIDToPose extends Command {
    private SwerveDrive swerve;
    private Odometry odometry;
    private Pose2d targetPose;
    private Pose2d robotPose;

    private PIDController xController;
    private PIDController yController;
    private AnglePIDController thetaController;

    private final FieldObject2d targetPose2d;

    public SwervePIDToPose(Pose2d targetPose){
        swerve = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();
        this.targetPose = targetPose;

        targetPose2d = odometry.getField().getObject("Swerve Target Pose");
        

        xController = new PIDController(1.0, 0, 0);
        yController = new PIDController(1.0, 0, 0);
        thetaController = new AnglePIDController(1.0, 0, 0);
        
        addRequirements(swerve, odometry);
    }

    boolean isAlignedX() {
        return Math.abs(targetPose.getX() - robotPose.getX()) < Settings.Swerve.Alignment.X_TOLERANCE.getAsDouble();
    }
    boolean isAlignedY() {
        return Math.abs(targetPose.getY() - robotPose.getY()) < Settings.Swerve.Alignment.Y_TOLERANCE.getAsDouble();
    }
    boolean isAlignedTheta() {
        return Math.abs(targetPose.getRotation().minus(robotPose.getRotation()).getRadians()) < Settings.Swerve.Alignment.THETA_TOLERANCE.getAsDouble();
    }
    
    @Override
    public void execute() {
        robotPose = odometry.getPose();

        targetPose2d.setPose(Robot.isBlue() ? targetPose : Field.transformToOppositeAlliance(targetPose));

        double outX = xController.calculate(robotPose.getX(), targetPose.getX());
        double outY = yController.calculate(robotPose.getY(), targetPose.getY());
        double outTheta = thetaController.update(Angle.fromRotation2d(robotPose.getRotation()), Angle.fromRotation2d(targetPose.getRotation()));

        ChassisSpeeds swerveChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            outX, 
            outY, 
            outTheta, 
            odometry.getRotation()
        );

        swerve.setChassisSpeeds(swerveChassisSpeeds);


        SmartDashboard.putNumber("Alignment/Target x", targetPose.getX());
        SmartDashboard.putNumber("Alignment/Target y", targetPose.getY());
        SmartDashboard.putNumber("Alignment/Target angle", targetPose.getRotation().getDegrees());


        SmartDashboard.putBoolean("Alignment/Is Aligned X", isAlignedX());
        SmartDashboard.putBoolean("Alignment/Is Aligned Y", isAlignedY());
        SmartDashboard.putBoolean("Alignment/Is Aligned Theta", isAlignedTheta());

        SmartDashboard.putNumber("Alignment/Target Velocity Robot Relative X (m per s)", outX);
        SmartDashboard.putNumber("Alignment/Target Velocity Robot Relative Y (m per s)", outY);
        SmartDashboard.putNumber("Alignment/Target Angular Velocity (rad per s)", outTheta);

    }


    @Override
    public boolean isFinished() {
        return isAlignedX() && isAlignedY() && isAlignedTheta();
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setChassisSpeeds(
            ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, Rotation2d.kZero)
        );
    }
}
