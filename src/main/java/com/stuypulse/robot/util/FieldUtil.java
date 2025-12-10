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
        return odometry.getPose().relativeTo(goaltag.getpose().toPose2d());
    }


    /**
     * Gets angle error to goal
     * @return angle error
     */
    public static Rotation2d getGoalAngleError() {
        return getGoalRelativePose().getRotation();
    }

    /**
     * Funtion to find pose to shoot at 
     * @return Pose2d object with current pose with angle correction or closest pose to shoot at if too close or too far.
     */
    public static Pose2d getShootPose() {
        if (odometry.getPose().getTranslation().getDistance(goaltag.getpose().getTranslation().toTranslation2d()) > com.stuypulse.robot.constants.Settings.HDSR.MAX_DISTANCE_METERS) {
            return new Pose2d(-getGoalAngleError().getSin() * com.stuypulse.robot.constants.Settings.HDSR.MAX_DISTANCE_METERS, getGoalAngleError().getCos() * com.stuypulse.robot.constants.Settings.HDSR.MAX_DISTANCE_METERS, getGoalAngleError());
        } else if (odometry.getPose().getTranslation().getDistance(goaltag.getpose().getTranslation().toTranslation2d()) < com.stuypulse.robot.constants.Settings.HDSR.MIN_DISTANCE_METERS) {
            return new Pose2d(-getGoalAngleError().getSin() * com.stuypulse.robot.constants.Settings.HDSR.MIN_DISTANCE_METERS, getGoalAngleError().getCos() * com.stuypulse.robot.constants.Settings.HDSR.MIN_DISTANCE_METERS, getGoalAngleError());
        } else {
            return odometry.getPose().rotateBy(getGoalAngleError());
        }
    }
}