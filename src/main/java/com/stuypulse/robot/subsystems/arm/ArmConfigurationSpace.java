package com.stuypulse.robot.subsystems.arm;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.spline.*;
import java.util.*;

import com.stuypulse.robot.constants.Constants;

public class ArmConfigurationSpace {

    private final Rotation2d shoulderMinAngle = Constants.Arm.SHOULDER_MIN_ANGLE;
    private final Rotation2d shoulderMaxAngle = Constants.Arm.SHOULDER_MAX_ANGLE;
    private final Rotation2d elbowMinAngle = Constants.Arm.ELBOW_MIN_ANGLE;
    private final Rotation2d elbowMaxAngle = Constants.Arm.ELBOW_MAX_ANGLE;

    private final double shoulderLength = Constants.Arm.SHOULDER_LENGTH;
    private final double elbowLength = Constants.Arm.ELBOW_LENGTH;

    private final boolean[][] obstacleGrid;
    private final double gridResolution = 100;
    
    // For visualization (Sim)
    private Translation2d currentEndEffectorPos;
    private Translation2d targetEndEffectorPos;
    private List<Translation2d> pathPoints = new ArrayList<>();

    public ArmConfigurationSpace() {
        this.obstacleGrid = new boolean[(int)gridResolution][(int)gridResolution];
    }

    // Convert joint angles to Cartesian space
    public Translation2d toCartesian(double theta1, double theta2) {
        double x = shoulderLength * Math.cos(theta1) + elbowLength * Math.cos(theta1 + theta2);
        double y = shoulderLength * Math.sin(theta1) + elbowLength * Math.sin(theta1 + theta2);
        return new Translation2d(x, y);
    }

    // Convert Cartesian to joint angles 
    public double[] toJointAngles(double x, double y) {
        double d = Math.sqrt(x*x + y*y);
        if (d > shoulderLength + elbowLength || d < Math.abs(shoulderLength - elbowLength)) {
            return null; // Invalid Target
        }
        
        double theta2 = Math.acos((x*x + y*y - shoulderLength*shoulderLength - elbowLength*elbowLength) 
                                / (2 * shoulderLength * elbowLength));
        double theta1 = Math.atan2(y, x) - Math.atan2(elbowLength * Math.sin(theta2), 
                                                    shoulderLength + elbowLength * Math.cos(theta2));
        
        return new double[]{theta1, theta2};
    }

    // Add obstacle in Cartesian space
    public void addObstacle(double x, double y) {
        double[] angles = toJointAngles(x, y);
        if (angles != null) {
            int theta1Idx = (int)((angles[0] - shoulderMinAngle.getDegrees()) / (shoulderMaxAngle.getDegrees() - shoulderMinAngle.getDegrees()) * gridResolution);
            int theta2Idx = (int)((angles[1] - elbowMinAngle.getDegrees()) / (elbowMaxAngle.getDegrees() - elbowMinAngle.getDegrees()) * gridResolution);
            
            if (theta1Idx >= 0 && theta1Idx < gridResolution && 
                theta2Idx >= 0 && theta2Idx < gridResolution) {
                obstacleGrid[theta1Idx][theta2Idx] = true;
            }
        }
    }

    // Check if configuration is valid (I think SLMath.clamp() got this? Double check)
    public boolean isValidConfiguration(double theta1, double theta2) {
    
        if (theta1 < shoulderMinAngle.getDegrees() || theta1 > shoulderMaxAngle.getDegrees() || 
            theta2 < elbowMinAngle.getDegrees() || theta2 > elbowMaxAngle.getDegrees()) {
            return false;
        }
        
        // Check obstacle grid
        int theta1Idx = (int)((theta1 - shoulderMinAngle.getDegrees()) / (shoulderMaxAngle.getDegrees() - shoulderMinAngle.getDegrees()) * gridResolution);
        int theta2Idx = (int)((theta2 - elbowMinAngle.getDegrees()) / (elbowMaxAngle.getDegrees() - elbowMinAngle.getDegrees()) * gridResolution);
        
        if (theta1Idx >= 0 && theta1Idx < gridResolution && 
            theta2Idx >= 0 && theta2Idx < gridResolution) {
            return !obstacleGrid[theta1Idx][theta2Idx];
        }
        return false;
    }
}