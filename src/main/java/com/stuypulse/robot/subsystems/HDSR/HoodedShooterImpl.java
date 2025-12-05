package com.stuypulse.robot.subsystems.HDSR;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.Robot;
import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HoodedShooterImpl extends HoodedShooter {
    private final TalonFX shooterMotor;

    private Translation2d[] distancexRPM;
    private InterpolatingDoubleTreeMap interpolator; 
    private Translation2d targetTranslation;
    private SmartNumber manualSetDistance;


    private final Odometry odometry;
    private final SwerveDrive swerve;
    private double targetDistance;
    private SmartNumber setRPMHDSR;

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
        swerve = SwerveDrive.getInstance();

        setRPMHDSR = new SmartNumber("HDSR/Settings/setRPM", getState().getTargetRPM());
        manualSetDistance = new SmartNumber("HDSR/Setable Values/Manual Set Distance", 4.2);

        targetTranslation = new Translation2d();
        targetDistance = 0;
    }


    /**
     * Method to get shooter velocity in RPM
     *@return Shooter velocity in RPM 
     */
    public double getCurrentVelocity() {
        return shooterMotor.getVelocity().getValueAsDouble()* 60;
    }

    public void setTargetTranslation(Translation2d targetTranslation) {
        this.targetTranslation = targetTranslation;
    }

    /**
     * finds the rpm to shoot to a target x meters away
\     * @return rpm needed to reach target 
     */
    public double RPMToDistanceInterpolation() {
        //prelim drive while shoot code
        Translation2d currentTranslation = odometry.getPose().getTranslation();
        double distanceToTarget = currentTranslation.getDistance(targetTranslation);

        //clamping
        distanceToTarget = Math.max(distanceToTarget, Settings.HDSR.MIN_DISTANCE_METERS);
        distanceToTarget = Math.min(distanceToTarget, Settings.HDSR.MAX_DISTANCE_METERS);

        SmartDashboard.putNumber("HDSR/interpolatorRPM", interpolator.get(distanceToTarget));

        return interpolator.get(distanceToTarget);
    } 

    @Override
    public void UpdateTargetDistance(double targetDistance) {
        this.targetDistance = targetDistance;
    }

    @Override 
    public double getTargetDistance() {
        return targetDistance;
    }

    
   
    @Override
    public void periodic() {
        super.periodic();

        switch (getState()) {
            case SHOOTRPM:
                    getState().setTargetRPM(() -> setRPMHDSR.getAsDouble());
                break;
            case INTERP:
                getState().setTargetRPM(() -> RPMToDistanceInterpolation());
                break;
            default:
                getState().setTargetRPM(() -> 0.0);
                break;
        } 

        this.targetDistance = manualSetDistance.getAsDouble();
        shooterMotor.setControl(new VelocityVoltage(getState().getTargetRPM() / 60.0).withSlot(0));
        
        SmartDashboard.putNumber("HDSR/currentVelocity", getCurrentVelocity());
        SmartDashboard.putNumber("HDSR/target velocity ", getState().getTargetRPM());
        SmartDashboard.putNumber("HDSR/Target distance hdsr", odometry.getPose().getTranslation().getDistance(targetTranslation));
    }
}

