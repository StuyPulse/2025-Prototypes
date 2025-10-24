package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

public interface Field {
    double WIDTH = Units.inchesToMeters(317.000); 
    double LENGTH = Units.inchesToMeters(690.876);

    public static Pose3d transformToOppositeAlliance(Pose3d pose) {
        Pose3d rotated = pose.rotateBy(new Rotation3d(0, 0, Math.PI));

        return new Pose3d(
            rotated.getTranslation().plus(new Translation3d(LENGTH, WIDTH, 0)),
            rotated.getRotation());
    }

    public static Pose2d transformToOppositeAlliance(Pose2d pose) {
        Pose2d rotated = pose.rotateBy(Rotation2d.fromDegrees(180));
        return new Pose2d(
            rotated.getTranslation().plus(new Translation2d(LENGTH, WIDTH)),
            rotated.getRotation());
    }

    /**** EMPTY FIELD POSES ****/
    Pose2d EMPTY_FIELD_POSE2D = new Pose2d(new Translation2d(-1, -1), new Rotation2d());
    Pose3d EMPTY_FIELD_POSE3D = new Pose3d(-1, -1, 0, new Rotation3d());

    public static void clearFieldObject(FieldObject2d fieldObject)  {
        fieldObject.setPose(EMPTY_FIELD_POSE2D);
    }
}
