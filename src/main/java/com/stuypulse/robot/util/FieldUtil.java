package com.stuypulse.robot.util;

import java.util.function.Supplier;

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
        Pose2d realativepose = new Pose2d(odometry.getPose().getX() - goaltag.getpose().getX(),
                odometry.getPose().getY() - goaltag.getpose().getY(),
                odometry.getPose().relativeTo(Tags.GoalTag.getpose().toPose2d()).getRotation());
        odometry.getField().getObject("tag").setPose(goaltag.getpose().toPose2d());
        odometry.getField().getObject("relative pose")
                .setPose(realativepose);
        return realativepose;
    }

    /**
     * Gets angle error to goal
     * 
     * @return angle error
     */
    public static Rotation2d getGoalAngleError() {
        Rotation2d rotationError;
        if (Math.signum(Math.atan(getGoalRelativePose().getX() / -getGoalRelativePose().getY())) == -1) {
            SmartDashboard.putBoolean("Alignment/Pose Gen/left of goal", true);
            rotationError = Rotation2d.kZero.minus(Rotation2d
                    .fromRadians(
                            Math.atan(getGoalRelativePose().getY() / -getGoalRelativePose().getX())));
        } else {
            SmartDashboard.putBoolean("Alignment/Pose Gen/right of goal", true);
            rotationError = Rotation2d.k180deg.plus(Rotation2d
                    .fromRadians(Math.atan(getGoalRelativePose().getY() / getGoalRelativePose().getX())));
        }
        SmartDashboard.putNumber("Alignment/Pose Gen/relative angle", rotationError.getDegrees());
        return rotationError;
    }

    /**
     * Funtion to find pose to shoot at
     * 
     * @return Pose2d object with current pose with angle correction or closest pose
     *         to shoot at if too close or too far.
     */
    public static Pose2d getShootPose() {
        odometry = Odometry.getInstance();

        // if robot "above" target pose, negate the coordinates
        double xNegate = (odometry.getPose().getY() > goaltag.getpose().getY()) ? -1.0 : 1.0;
        double yNegate = (odometry.getPose().getY() < goaltag.getpose().getY()) ? -1.0 : 1.0;

        if (odometry.getPose().getTranslation().getDistance(goaltag.getpose().getTranslation()
                .toTranslation2d()) > com.stuypulse.robot.constants.Settings.HDSR.MAX_DISTANCE_METERS) {
            // code to run if robot is out of shoot range
            Rotation2d displacementAngle = Rotation2d
                    .fromRadians(Math.atan(getGoalRelativePose().getX() / -getGoalRelativePose().getY()));
            SmartDashboard.putNumber("Alignment/Pose Gen/ displacement angle", displacementAngle.getDegrees());
            SmartDashboard.putNumber("Alignment/Pose Gen/ displacement sin", displacementAngle.getSin());
            SmartDashboard.putNumber("Alignment/Pose Gen/Displacement Cos", displacementAngle.getCos());
            SmartDashboard.putBoolean("Alignment/Pose Gen/To far from goal", true);
            // code that generates robot pose w/o rotation
            Pose2d relativePose = new Pose2d(
                    goaltag.getpose().getX()
                            + (xNegate * displacementAngle.getSin()
                                    * com.stuypulse.robot.constants.Settings.HDSR.MAX_DISTANCE_METERS),
                    goaltag.getpose().getY() + (yNegate * displacementAngle.getCos()
                            * com.stuypulse.robot.constants.Settings.HDSR.MAX_DISTANCE_METERS),
                    Rotation2d.kZero);
            // final pose with correct rotation, adds 180 degrees if robot is right of goal
            return (Math.signum(Math.atan((relativePose.getY() - goaltag.getpose().getY())
                    / (relativePose.getX() - goaltag.getpose().getX()))) == -1)
                            ? relativePose.rotateAround(relativePose.getTranslation(),
                                    Rotation2d.k180deg
                                            .plus(Rotation2d.fromRadians(
                                                    Math.atan((relativePose.getY() - goaltag.getpose().getY())
                                                            / (relativePose.getX() - goaltag.getpose().getX())))))
                            : relativePose.rotateAround(relativePose.getTranslation(),
                                    Rotation2d.fromRadians(Math.atan(((relativePose.getY() - goaltag.getpose().getY())
                                            / (relativePose.getY() - goaltag.getpose().getX())))));
        } else if (odometry.getPose().getTranslation().getDistance(goaltag.getpose().getTranslation()
                .toTranslation2d()) < com.stuypulse.robot.constants.Settings.HDSR.MIN_DISTANCE_METERS) {
            // code to run if robot is too close to goal
            Rotation2d displacementAngle = Rotation2d
                    .fromRadians(Math.atan(getGoalRelativePose().getX() / -getGoalRelativePose().getY()));
            SmartDashboard.putNumber("Alignment/Pose Gen/ displacement angle", displacementAngle.getDegrees());
            SmartDashboard.putNumber("Alignment/Pose Gen/ displacement sin", displacementAngle.getSin());
            SmartDashboard.putNumber("Alignment/Pose Gen/Displacement Cos", displacementAngle.getCos());
            SmartDashboard.putBoolean("Alignment/Pose Gen/To close to goal", true);
            // code that generates robot pose w/o rotation
            Pose2d relativePose = new Pose2d(
                    goaltag.getpose().getX()
                            + (xNegate * displacementAngle.getSin()
                                    * com.stuypulse.robot.constants.Settings.HDSR.MIN_DISTANCE_METERS),
                    goaltag.getpose().getY() + (yNegate * displacementAngle.getCos()
                            * com.stuypulse.robot.constants.Settings.HDSR.MIN_DISTANCE_METERS),
                    Rotation2d.kZero);
            // final pose with correct rotation, adds 180 degrees if robot is right of goal
            return (Math.signum(Math.atan((relativePose.getY() - goaltag.getpose().getY())
                    / (relativePose.getX() - goaltag.getpose().getX()))) == -1)
                            ? relativePose.rotateAround(relativePose.getTranslation(),
                                    Rotation2d.k180deg
                                            .plus(Rotation2d.fromRadians(
                                                    Math.atan((relativePose.getY() - goaltag.getpose().getY())
                                                            / (relativePose.getX() - goaltag.getpose().getX())))))
                            : relativePose.rotateAround(relativePose.getTranslation(),
                                    Rotation2d.fromRadians(Math.atan(((relativePose.getY() - goaltag.getpose().getY())
                                            / (relativePose.getY() - goaltag.getpose().getX())))));
        } else {
            // default case (code to run if robot is in range)
            SmartDashboard.putBoolean("Alignment/Pose Gen/In goal range", true);
            return (xNegate == -1) ? new Pose2d(odometry.getPose().getTranslation(),
                    getGoalAngleError().plus(Rotation2d.k180deg))
                    : new Pose2d(odometry.getPose().getTranslation(), getGoalAngleError());
        }
    }
}