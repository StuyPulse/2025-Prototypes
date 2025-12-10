package com.stuypulse.robot.util;

import org.dyn4j.dynamics.Settings;
import org.dyn4j.geometry.Rotation;
import org.opencv.core.Mat;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Constants.Tags;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldUtil {
    private static Odometry odometry;
    private static Tags goaltag;
    static {
        odometry = Odometry.getInstance();
        goaltag = Tags.GoalTag;
    }

    public static Pose2d fieldTransform(Pose2d pose) {
        pose = Robot.isBlue() ? pose : Field.transformToOppositeAlliance(pose);
        return pose;
    }

    
    public static Pose2d getGoalRelativePose() {
        odometry = Odometry.getInstance();
        odometry.getField().getObject("tag").setPose(goaltag.getpose().toPose2d());
        odometry.getField().getObject("relative pose").setPose(new Pose2d(goaltag.getpose().toPose2d().minus(odometry.getPose()).getTranslation(), odometry.getPose().relativeTo(Tags.GoalTag.getpose().toPose2d()).getRotation()));
        return new Pose2d(goaltag.getpose().toPose2d().minus(odometry.getPose()).getTranslation(), odometry.getPose().relativeTo(Tags.GoalTag.getpose().toPose2d()).getRotation());
    }


    /**
     * Gets angle error to goal
     * @return angle error
     */
    public static Rotation2d getGoalAngleError() {
        SmartDashboard.putNumber("Goal Alignment/ relative angle", getGoalRelativePose().getRotation().getDegrees());
        return Rotation2d.fromRadians(Math.atan(getGoalRelativePose().getX()/-getGoalRelativePose().getY()));
    }

    /**
     * Funtion to find pose to shoot at 
     * @return Pose2d object with current pose with angle correction or closest pose to shoot at if too close or too far.
     */
    public static Pose2d getShootPose() {
        odometry = Odometry.getInstance();
        if (odometry.getPose().getTranslation().getDistance(goaltag.getpose().getTranslation().toTranslation2d()) > com.stuypulse.robot.constants.Settings.HDSR.MAX_DISTANCE_METERS) {
            SmartDashboard.putBoolean("Alignment/to far from goal",true);
            return new Pose2d(Math.abs(goaltag.getpose().getX() - (-getGoalAngleError().getSin() * com.stuypulse.robot.constants.Settings.HDSR.MAX_DISTANCE_METERS)), Math.abs(goaltag.getpose().getY() - getGoalAngleError().getCos() * com.stuypulse.robot.constants.Settings.HDSR.MAX_DISTANCE_METERS), Rotation2d.fromRadians(Math.atan(getGoalRelativePose().getY()/getGoalRelativePose().getX())));        } else if (odometry.getPose().getTranslation().getDistance(goaltag.getpose().getTranslation().toTranslation2d()) < com.stuypulse.robot.constants.Settings.HDSR.MIN_DISTANCE_METERS) {
            SmartDashboard.putBoolean("Alignment/to close to goal",true);
            // return new Pose2d(
            //     goaltag.getpose().toPose2d()
            //         .minus(new Pose2d(-getGoalAngleError().getSin() * com.stuypulse.robot.constants.Settings.HDSR.MIN_DISTANCE_METERS, 
            //         getGoalAngleError().getCos() * com.stuypulse.robot.constants.Settings.HDSR.MIN_DISTANCE_METERS, 
            //         Rotation2d.fromRadians(Math.atan(getGoalRelativePose().getY()/getGoalRelativePose().getX()))))
            //      .getTranslation(), 
            //     Rotation2d.fromRadians(Math.atan(getGoalRelativePose().getY()/getGoalRelativePose().getX())) );
            return new Pose2d(Math.abs(goaltag.getpose().getX() - (-getGoalAngleError().getSin() * com.stuypulse.robot.constants.Settings.HDSR.MIN_DISTANCE_METERS)), Math.abs(goaltag.getpose().getY() - getGoalAngleError().getCos() * com.stuypulse.robot.constants.Settings.HDSR.MIN_DISTANCE_METERS), Rotation2d.fromRadians(Math.atan(getGoalRelativePose().getY()/getGoalRelativePose().getX())));
        } else {
            SmartDashboard.putBoolean("Alignment/in goal range",true);
            return new Pose2d(odometry.getPose().getX(), odometry.getPose().getY(), (Rotation2d.fromRadians(Math.atan(getGoalRelativePose().getY()/getGoalRelativePose().getX()))));
        }
    }
}