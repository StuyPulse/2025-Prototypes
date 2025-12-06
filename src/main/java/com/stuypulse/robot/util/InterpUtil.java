package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class InterpUtil {
    private static InterpolatingDoubleTreeMap levelDistanceInterp;
    private static InterpolatingDoubleTreeMap goalinterpolation;
    private static Odometry odometry;
    
    public InterpUtil() {
        levelDistanceInterp = new InterpolatingDoubleTreeMap();
        for(Translation2d cords : Settings.HDSR.levelDistanceXrpm) {
            levelDistanceInterp.put(cords.getX(), cords.getY());
        }

        goalinterpolation = new InterpolatingDoubleTreeMap();
        for (Translation2d data : Settings.HDSR.goalDistanceXrpm ) {
            goalinterpolation.put(data.getX(), data.getY());
        }

        odometry = Odometry.getInstance();
    }

    public static double getLevelDistanceInterp(Translation2d targetTranslation) {
        targetTranslation = FieldUtil.fieldTransform(new Pose2d(targetTranslation.getX(), targetTranslation.getY(), Rotation2d.kZero)).getTranslation();
        double targetDistance = odometry.getPose().getTranslation().getDistance(targetTranslation);

        return levelDistanceInterp.get(targetDistance);
    }

    public static double getGoalDistanceInterp(Translation2d targetTranslation) {
        targetTranslation = FieldUtil.fieldTransform(new Pose2d(targetTranslation.getX(), targetTranslation.getY(), Rotation2d.kZero)).getTranslation();
        double targetDistance = odometry.getPose().getTranslation().getDistance(targetTranslation);

        return goalinterpolation.get(targetDistance);
    }


}