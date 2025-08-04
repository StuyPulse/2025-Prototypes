package com.stuypulse.robot.subsystems.arm;

import edu.wpi.first.math.geometry.*;

import java.util.*;

import com.stuypulse.robot.constants.Constants;

public class ArmConfigurationSpace {

    private final Rotation2d shoulderMinAngle = Constants.Arm.SHOULDER_MIN_ANGLE;
    private final Rotation2d shoulderMaxAngle = Constants.Arm.SHOULDER_MAX_ANGLE;
    private final Rotation2d elbowMinAngle = Constants.Arm.ELBOW_MIN_ANGLE;
    private final Rotation2d elbowMaxAngle = Constants.Arm.ELBOW_MAX_ANGLE;

    private final double shoulderLength = Constants.Arm.SHOULDER_LENGTH;
    private final double elbowLength = Constants.Arm.ELBOW_LENGTH;

    private final boolean[][] obstaclePoint; 
    private final double gridResolution = 100; // 100x100 grid
    
    // For visualization (Sim)
    private Translation2d currentEndEffectorPos;
    private Translation2d targetEndEffectorPos;
    private List<Translation2d> pathPoints = new ArrayList<>();

    public ArmConfigurationSpace() {
        this.obstaclePoint = new boolean[(int)gridResolution][(int)gridResolution];
    }

    // Convert joint angles to Cartesian space
    public Translation2d toCartesian(Rotation2d theta1, Rotation2d theta2) {
        double x = shoulderLength * Math.cos(theta1.getRadians()) + elbowLength * Math.cos(theta1.getRadians() + theta2.getRadians());
        double y = shoulderLength * Math.sin(theta1.getRadians()) + elbowLength * Math.sin(theta1.getRadians() + theta2.getRadians());
        return new Translation2d(x, y);
    }

    // Convert Cartesian to joint angles
    // Generate two possible configurations?!?!? 
    public Rotation2d[] toJointAngles(double x, double y) {
        double d = Math.sqrt(x*x + y*y);
        if (d > shoulderLength + elbowLength || d < Math.abs(shoulderLength - elbowLength)) {
            return null; // Invalid Target
        }
        
        Rotation2d theta2 = new Rotation2d(Math.acos((x*x + y*y - shoulderLength*shoulderLength - elbowLength*elbowLength)
                                / (2 * shoulderLength * elbowLength)));
        Rotation2d theta1 = new Rotation2d(Math.atan2(y, x) - Math.atan2(elbowLength * Math.sin(theta2.getRadians()), 
                                                    shoulderLength + elbowLength * Math.cos(theta2.getRadians())));
        
        return new Rotation2d[]{theta1, theta2};
    }

    // Add obstacle in Cartesian space
    public void addObstacle(double x, double y) {
        Rotation2d[] angles = toJointAngles(x, y);

        if (angles != null) {
            int theta1Idx = (int)((angles[0].getRadians() - shoulderMinAngle.getRadians()) / (shoulderMaxAngle.getRadians() - shoulderMinAngle.getRadians()) * gridResolution);
            int theta2Idx = (int)((angles[1].getRadians() - elbowMinAngle.getRadians()) / (elbowMaxAngle.getRadians() - elbowMinAngle.getRadians()) * gridResolution);
            
            if (theta1Idx >= 0 && theta1Idx < gridResolution && 
                theta2Idx >= 0 && theta2Idx < gridResolution) {
                obstaclePoint[theta1Idx][theta2Idx] = true;
            }
        }
    }

    // Check if configuration is valid (I think SLMath.clamp() got this? Double check)
    public boolean isValidConfiguration(Rotation2d theta1, Rotation2d theta2) {
    
        if (theta1.getRadians() < shoulderMinAngle.getRadians() || theta1.getRadians() > shoulderMaxAngle.getRadians() || 
            theta2.getRadians() < elbowMinAngle.getRadians() || theta2.getRadians() > elbowMaxAngle.getRadians()) {
            return false;
        }
        
        // Check obstacle grid
        int theta1Idx = (int)((theta1.getRadians() - shoulderMinAngle.getRadians()) / (shoulderMaxAngle.getRadians() - shoulderMinAngle.getRadians()) * gridResolution);
        int theta2Idx = (int)((theta2.getRadians() - elbowMinAngle.getRadians()) / (elbowMaxAngle.getRadians() - elbowMinAngle.getRadians()) * gridResolution);
        
        if (theta1Idx >= 0 && theta1Idx < gridResolution && 
            theta2Idx >= 0 && theta2Idx < gridResolution) {
            return !obstaclePoint[theta1Idx][theta2Idx];
        }
        return false;
    }
}