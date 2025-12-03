package com.stuypulse.robot.util;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Field;

import edu.wpi.first.math.geometry.Pose2d;

public class FieldUtil {

    public static Pose2d fieldTransform(Pose2d pose) {
        pose = Robot.isBlue() ? pose : Field.transformToOppositeAlliance(pose);
        return pose;
    }
    
}