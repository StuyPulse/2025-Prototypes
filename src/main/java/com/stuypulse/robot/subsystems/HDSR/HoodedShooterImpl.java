package com.stuypulse.robot.subsystems.HDSR;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.odometry.OdometryImpl;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HoodedShooterImpl extends HoodedShooter {
    private final TalonFX shooterMotor;
    private SmartNumber setRPM;
    private SmartNumber setDistanceToTarget;

    private Translation2d[] distancexRPM;
    private InterpolatingDoubleTreeMap interpolator; 
    private final Odometry odometry;

    public HoodedShooterImpl() {
        super();
        
        interpolator = new InterpolatingDoubleTreeMap();
        distancexRPM = Settings.HDSR.distanceXrpm;
        for (Translation2d point : distancexRPM) {
            interpolator.put(point.getX(), point.getY());
        }

        shooterMotor = new TalonFX(Ports.HDSR.SHOOTER_MOTOR, "swerve");
                
        Motors.SHOOTER_MOTOR_CONFIG.configure(shooterMotor);

        odometry = Odometry.getInstance();
        setRPM = new SmartNumber("HDSR/Setable Values/ setRPM", getState().getTargetRPM());
        setDistanceToTarget = new SmartNumber("HDSR/Setable Values/ setdistancetotarget", 0);
    }


    /**
     * Method to get shooter velocity in RPM
     *@return Shooter velocity in RPM 
     */
    public double getCurrentVelocity() {
        return shooterMotor.getVelocity().getValueAsDouble()* 60;
    }

    /**
     * finds the rpm to shoot to a target x meters away
     * @param targettranslation represents the target translation to shoot to
     * @return rpm needed to reach distance 
     */
    public double RPMToDistanceInterpolation(Translation2d targetTranslation) {
        //prelim drive while shoot code
        Translation2d currentTranslation = odometry.getPose().getTranslation();
        double distanceToTarget = currentTranslation.getDistance(targetTranslation);
        // distanceMeters = distanceMeters - SwerveDrive.getInstance().getChassisSpeeds().vxMetersPerSecond / 50;

        //clamping
        distanceToTarget = Math.max(distanceToTarget, 1);
        distanceToTarget = Math.min(distanceToTarget, 7);

        SmartDashboard.putNumber("HDSR/interpolatorRPM", interpolator.get(distanceToTarget));

        return interpolator.get(distanceToTarget);
    }
   
    @Override
    public void periodic() {
        super.periodic();

        if (getState() == HoodState.DEFAULT) getState().setTargetRPM(RPMToDistanceInterpolation(new Translation2d(odometry.getPose().getX() + setDistanceToTarget.getAsDouble(), odometry.getPose().getY())));

        // hoodMotor.setControl(new PositionVoltage(getState().getTargetAngle().getRotations()));
        shooterMotor.setControl(new VelocityVoltage(setRPM.doubleValue() / 60.0).withSlot(0));
        
        SmartDashboard.putNumber("HDSR/currentVelocity", getCurrentVelocity());
        SmartDashboard.putNumber("HDSR/ target velocity ", getState().getTargetRPM());
        // SmartDashboard.putNumber("HDSR/currentAngle", getCurrentAngle().getDegrees());
    }
}

