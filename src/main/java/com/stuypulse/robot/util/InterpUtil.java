package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Constants.Tags;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class InterpUtil {
    private static InterpolatingDoubleTreeMap levelDistanceInterp = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap goalinterpolation = new InterpolatingDoubleTreeMap();
    private static Odometry odometry = Odometry.getInstance();

    static {
        for (Translation2d data : Settings.HDSR.goalDistanceXrpm ) {
            levelDistanceInterp.put(data.getX(), data.getY());
        }

        for (Translation2d data : Settings.HDSR.goalDistanceXrpm ) {
            goalinterpolation.put(data.getX(), data.getY());
        }
    }

    public static double getLevelDistanceInterp(Translation2d targetTranslation) {
        targetTranslation = FieldUtil.fieldTransform(new Pose2d(targetTranslation.getX(), targetTranslation.getY(), Rotation2d.kZero)).getTranslation();
        double targetDistance = odometry.getPose().getTranslation().getDistance(targetTranslation);

        return levelDistanceInterp.get(targetDistance);
    }

    public static double getGoalDistanceInterp() {
        double targetDistance = odometry.getPose().getTranslation().getDistance(Tags.GoalTag.getpose().getTranslation().toTranslation2d());
        return goalinterpolation.get(targetDistance);
    }

}