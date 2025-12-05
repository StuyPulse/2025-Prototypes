package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwervePIDToPose extends Command {
    private SwerveDrive drive;
    private Odometry odometry;
    private Translation2d targetPose;
    private PIDController xController;
    private PIDController yController;
    private AnglePIDController thetaController;

    public SwervePIDToPose(Translation2d targetPose){
        drive = SwerveDrive.getInstance();
        odometry = Odometry.getInstance();
        this.targetPose = targetPose;
        

        xController = new PIDController(1.0, 0, 0);
        yController = new PIDController(1.0, 0, 0);
        thetaController = new AnglePIDController(1.0, 0, 0);
        
        addRequirements(drive, odometry);
    }
    
    @Override
    public void execute() {
        Pose2d pose = odometry.getPose();

        double outX = xController.calculate(pose.getX(), targetPose.getX());
        double outY = yController.calculate(pose.getY(), targetPose.getY());
        double outTheta = thetaController.update(Angle.fromRotation2d(pose.getRotation()), Angle.kZero);
    }
    
}
