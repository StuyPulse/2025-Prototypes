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
import edu.wpi.first.math.geometry.Translation2d;
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

        /**
         * Gets goal relative translation using current robot pose
         * 
         * @return current pose relative to the goal
         */
        public static Translation2d getGoalRelativeTranslation() {
                odometry = Odometry.getInstance();
                // translation relative to goal
                Translation2d realativeTranslation = new Translation2d(
                                odometry.getPose().getX() - goaltag.getpose().getX(),
                                odometry.getPose().getY() - goaltag.getpose().getY());
                odometry.getField().getObject("tag").setPose(goaltag.getpose().toPose2d());
                odometry.getField().getObject("relative pose")
                                .setPose(new Pose2d(realativeTranslation, Rotation2d.kZero));
                return realativeTranslation;
        }

        /**
         * finds the translation of a pose relative to a given point
         * 
         * @param translation the translation of the point to find relative to the goal
         * @return translation of input relative to goal
         */
        public static Translation2d getGoalRelativeTranslation(Translation2d translation) {
                // translation relative to goal
                Translation2d realativeTranslation = new Translation2d(
                                translation.getX() - goaltag.getpose().getX(),
                                translation.getY() - goaltag.getpose().getY());
                odometry.getField().getObject("tag").setPose(goaltag.getpose().toPose2d());
                odometry.getField().getObject("relative pose")
                                .setPose(new Pose2d(realativeTranslation, Rotation2d.kZero));
                return realativeTranslation;
        }

        /**
         * gets rotation needed to point directly at goal given a point
         * 
         * @return required rotation
         */
        public static Rotation2d getGoalAngleError(Translation2d translation) {
                Translation2d goalRelativeTranslation = getGoalRelativeTranslation(translation);
                Rotation2d rotationError;
                if (Math.signum(Math.atan(goalRelativeTranslation.getX() / -goalRelativeTranslation.getY())) == -1) {
                        SmartDashboard.putBoolean("Alignment/Pose Gen/left of goal", true);
                        rotationError = Rotation2d.kZero.minus(Rotation2d
                                        .fromRadians(
                                                        Math.atan(goalRelativeTranslation.getY()
                                                                        / -goalRelativeTranslation.getX())));
                } else {
                        SmartDashboard.putBoolean("Alignment/Pose Gen/right of goal", true);
                        rotationError = Rotation2d.k180deg.plus(Rotation2d
                                        .fromRadians(Math.atan(goalRelativeTranslation.getY()
                                                        / goalRelativeTranslation.getX())));
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
                double yNegate = -xNegate;

                if (odometry.getPose().getTranslation().getDistance(goaltag.getpose().getTranslation()
                                .toTranslation2d()) > com.stuypulse.robot.constants.Settings.HDSR.MAX_DISTANCE_METERS) {
                        // code to run if robot is out of shoot range
                        Rotation2d displacementAngle = Rotation2d
                                        .fromRadians(Math.atan(getGoalRelativeTranslation().getX()
                                                        / -getGoalRelativeTranslation().getY()));
                        SmartDashboard.putNumber("Alignment/Pose Gen/ displacement angle",
                                        displacementAngle.getDegrees());
                        SmartDashboard.putNumber("Alignment/Pose Gen/ displacement sin", displacementAngle.getSin());
                        SmartDashboard.putNumber("Alignment/Pose Gen/Displacement Cos", displacementAngle.getCos());
                        SmartDashboard.putBoolean("Alignment/Pose Gen/To far from goal", true);
                        // code that generates robot translation
                        Translation2d relativeTranslation = new Translation2d(
                                        goaltag.getpose().getX()
                                                        + (xNegate * displacementAngle.getSin()
                                                                        * com.stuypulse.robot.constants.Settings.HDSR.MAX_DISTANCE_METERS),
                                        goaltag.getpose().getY() + (yNegate * displacementAngle.getCos()
                                                        * com.stuypulse.robot.constants.Settings.HDSR.MAX_DISTANCE_METERS));
                        return new Pose2d(relativeTranslation, getGoalAngleError(relativeTranslation));
                } else if (odometry.getPose().getTranslation().getDistance(goaltag.getpose().getTranslation()
                                .toTranslation2d()) < com.stuypulse.robot.constants.Settings.HDSR.MIN_DISTANCE_METERS) {
                        // code to run if robot is too close to goal
                        Rotation2d displacementAngle = Rotation2d
                                        .fromRadians(Math.atan(getGoalRelativeTranslation().getX()
                                                        / -getGoalRelativeTranslation().getY()));
                        SmartDashboard.putNumber("Alignment/Pose Gen/ displacement angle",
                                        displacementAngle.getDegrees());
                        SmartDashboard.putNumber("Alignment/Pose Gen/ displacement Sin", displacementAngle.getSin());
                        SmartDashboard.putNumber("Alignment/Pose Gen/Displacement Cos", displacementAngle.getCos());
                        SmartDashboard.putBoolean("Alignment/Pose Gen/To close to goal", true);
                        // code that generates robot translation
                        Translation2d relativeTranslation = new Translation2d(
                                        goaltag.getpose().getX()
                                                        + (xNegate * displacementAngle.getSin()
                                                                        * com.stuypulse.robot.constants.Settings.HDSR.MIN_DISTANCE_METERS),
                                        goaltag.getpose().getY() + (yNegate * displacementAngle.getCos()
                                                        * com.stuypulse.robot.constants.Settings.HDSR.MIN_DISTANCE_METERS));
                        return new Pose2d(relativeTranslation, getGoalAngleError(relativeTranslation));
                } else {
                        // default case (code to run if robot is in range)
                        SmartDashboard.putBoolean("Alignment/Pose Gen/In goal range", true);
                        return new Pose2d(odometry.getPose().getTranslation(),
                                        getGoalAngleError(odometry.getPose().getTranslation()));
                }
        }
}