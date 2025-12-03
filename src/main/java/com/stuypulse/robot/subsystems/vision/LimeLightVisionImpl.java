package com.stuypulse.robot.subsystems.vision;

import javax.security.auth.login.FailedLoginException;

import org.dyn4j.dynamics.Settings;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Constants.Cameras;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.Vision.LimelightHelpers;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class LimeLightVisionImpl extends LimelightVision {
    private final Odometry odometry;
    private final Cameras camera;
    private boolean apriltagDetected, doRejectUpdate;
    private final Matrix<N3, N1> visionStdDevs;

    public LimeLightVisionImpl() {
        odometry = Odometry.getInstance();
        camera = Constants.Cameras.Limelight;
        doRejectUpdate = false;
        apriltagDetected = false;
        visionStdDevs = VecBuilder.fill(1, 1, .3);

        for (Cameras camera : Constants.Cameras.values()) {
                   Pose3d robotRelativePose = camera.getLocation();
            LimelightHelpers.setCameraPose_RobotSpace(
                    camera.getName(),
                    robotRelativePose.getX(),
                    -robotRelativePose.getY(),
                    robotRelativePose.getZ(),
                    Units.radiansToDegrees(robotRelativePose.getRotation().getX()),
                    Units.radiansToDegrees(robotRelativePose.getRotation().getY()),
                    Units.radiansToDegrees(robotRelativePose.getRotation().getZ()));
        }
    }



    private void updatePoseEstimatorVisionMeasurement() {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight"); 

            if(mt1 == null) return;

            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                if(mt1.rawFiducials[0].ambiguity > .7) {
                    doRejectUpdate = true;
                }
                if(mt1.rawFiducials[0].distToCamera > 3) {
                    doRejectUpdate = true;
                }
            }

            if (mt1.tagCount == 0) {
                doRejectUpdate = true;
            }

            if(!doRejectUpdate) {
                apriltagDetected = true;

                odometry.updateVisionMeasurement(visionStdDevs, mt1.pose, mt1.timestampSeconds);
            }
        }
    
}
