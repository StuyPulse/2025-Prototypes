package com.stuypulse.robot.subsystems.wiffleWorksShooter;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;

public class WiffleWorksShooterImpl extends WiffleWorksShooter {
    private final TalonFX shooterMotor;
    double setRPMvalue = 3000;
    private SmartNumber setRPM = new SmartNumber("HDSR/ setRPM", getState().getTargetRPM());
    private boolean hasBall;
    private Translation2d targetTranlation;
    private final Translation2d[] interpolationdata;
    private InterpolatingDoubleTreeMap interpolator;
    private double targetRPM;

    public WiffleWorksShooterImpl() {
        super();
        shooterMotor = new TalonFX(Ports.HDSR.SHOOTER_MOTOR, "swerve");

        interpolationdata = Settings.WiffleWorksShooter.distanceXrpm;
        for (Translation2d datapoint : interpolationdata) {
            interpolator.put(datapoint.getX(), datapoint.getY());
        }
        
        hasBall = false;
        
        Motors.SHOOTER_MOTOR_CONFIG.configure(shooterMotor);

        targetTranlation = new Translation2d();

    }

    public double getCurrentVelocity() {
        return shooterMotor.getVelocity().getValueAsDouble()* 60;
    }

    public double RPMToDistanceInterpolation() {
        double targetDistance = Odometry.getInstance().getPose().getTranslation().getDistance(targetTranlation);
        targetDistance = Math.min(targetDistance, Settings.WiffleWorksShooter.MAX_DISTANCE_METERS);
        targetDistance = Math.min(targetDistance, Settings.WiffleWorksShooter.MIN_DISTANCE_METERS);

        return interpolator.get(targetDistance);
    }

    @Override
    public void setTargetTranslation(Translation2d targetTranslation) {
        this.targetTranlation = targetTranslation;
    }
   
    @Override
    public void periodic() {
        super.periodic();

        if (getState() == ShooterState.SHOOT) {
            getState().setTargetRPM(RPMToDistanceInterpolation());
        } else if (getState() == ShooterState.DEFAULT) {
            getState().setTargetRPM(setRPM.getAsDouble());
        } else {
            getState().setTargetRPM(0);
        }

        shooterMotor.setControl(new VelocityVoltage(getState().getTargetRPM() / 60.0).withSlot(0));

        SmartDashboard.putNumber("HDSR/currentVelocity", getCurrentVelocity());
        SmartDashboard.putNumber("HDSR/ target velocity ", getState().getTargetRPM());
    }
}
