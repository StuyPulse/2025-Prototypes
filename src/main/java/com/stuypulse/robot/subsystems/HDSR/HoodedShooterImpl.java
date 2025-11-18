package com.stuypulse.robot.subsystems.HDSR;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HoodedShooterImpl extends HoodedShooter {
    private final TalonFX shooterMotor;
    private SmartNumber setRPM = new SmartNumber("HDSR/ setRPM", getState().getTargetRPM());
    private Translation2d[] distancexRPM;
    private InterpolatingDoubleTreeMap interpolator; 

    public HoodedShooterImpl() {
        super();
        
        interpolator = new InterpolatingDoubleTreeMap();
        distancexRPM = Settings.HDSR.distanceXrpm;
        for (Translation2d point : distancexRPM) {
            interpolator.put(point.getX(), point.getY());
        }

        shooterMotor = new TalonFX(Ports.HDSR.SHOOTER_MOTOR, "swerve");
                
        Motors.SHOOTER_MOTOR_CONFIG.configure(shooterMotor);
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
     * @param distanceMeters distance to target in meters, must be between 1 and 6
     * @return rpm needed to reach distance 
     */
    public double RPMToDistanceInterpolation(double distanceMeters) {
        //prelim drive while shoot code
        distanceMeters = distanceMeters - SwerveDrive.getInstance().getChassisSpeeds().vxMetersPerSecond / 50;

        //clamping
        distanceMeters = Math.max(distanceMeters, 1);
        distanceMeters = Math.min(distanceMeters, 6);

        SmartDashboard.putNumber("HDSR/interpolatorRPM", interpolator.get(distanceMeters));

        return interpolator.get(distanceMeters);
    }
   
    @Override
    public void periodic() {
        super.periodic();

        if (getState() == HoodState.DEFAULT) getState().setTargetRPM(RPMToDistanceInterpolation(getDistanceToTarget()));

        // hoodMotor.setControl(new PositionVoltage(getState().getTargetAngle().getRotations()));
        shooterMotor.setControl(new VelocityVoltage(setRPM.doubleValue() / 60.0).withSlot(0));
        
        SmartDashboard.putNumber("HDSR/currentVelocity", getCurrentVelocity());
        SmartDashboard.putNumber("HDSR/ target velocity ", getState().getTargetRPM());
        // SmartDashboard.putNumber("HDSR/currentAngle", getCurrentAngle().getDegrees());
    }
}

