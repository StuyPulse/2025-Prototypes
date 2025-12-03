package com.stuypulse.robot.util;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class InterpUtil {
    private static InterpolatingDoubleTreeMap distanceInterp;
    private static  Odometry odometry;
    
    public InterpUtil() {
        distanceInterp = new InterpolatingDoubleTreeMap();
        for(Translation2d cords : Settings.HDSR.distanceXrpm) {
            distanceInterp.put(cords.getX(), cords.getY());
        }
        odometry = Odometry.getInstance();


    }

    public static double getDistanceInterp(Translation2d targetTranslation) {
        targetTranslation = FieldUtil.fieldTransform(new Pose2d(targetTranslation.getX(), targetTranslation.getY(), Rotation2d.kZero)).getTranslation();
        double targetDistance = odometry.getPose().getTranslation().getDistance(targetTranslation);
        return distanceInterp.get(targetDistance);
    }


}