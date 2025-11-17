package com.stuypulse.robot.subsystems.HDSR;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HoodedShooterImpl extends HoodedShooter {
    // private final TalonFX hoodMotor;
    // private final TalonFX rollerMotor;
    private final TalonFX shooterMotor;
    double setRPMvalue = 3000;
   private SmartNumber setRPM = new SmartNumber("HDSR/ setRPM", getState().getTargetRPM());
    private boolean hasBall;
    private final Translation2d[] distancexRPM;
    private InterpolatingDoubleTreeMap interpolator; 

    public HoodedShooterImpl() {
        super();
        interpolator = new InterpolatingDoubleTreeMap();
        distancexRPM = Settings.HDSR.distanceXrpm;

        for (Translation2d point : distancexRPM) {
            interpolator.put(point.getX(), point.getY());
        }

        // hoodMotor = new TalonFX(Ports.HDSR.HOOD_MOTOR);
        // rollerMotor = new TalonFX(Ports.HDSR.ROLLER_MOTOR);
        shooterMotor = new TalonFX(Ports.HDSR.SHOOTER_MOTOR, "swerve");
        
        hasBall = false;
        
        Motors.SHOOTER_MOTOR_CONFIG.configure(shooterMotor);
        // Motors.ROLLER_MOTOR_CONFIG.configure(rollerMotor);
        // Motors.HOOD_MOTOR_CONFIG.configure(hoodMotor);
    }

    // public Rotation2d getCurrentAngle() {
    //     return Rotation2d.fromRotations(hoodMotor.getPosition().getValueAsDouble());
    // }

    public double getCurrentVelocity() {
        return shooterMotor.getVelocity().getValueAsDouble()* 60;
    }

    // public void setRollerSpeeds(double speed){
    //     rollerMotor.set(speed);
    // }

    public double RPMToDistanceInterpolation(double distanceMeters) {
        distanceMeters = Math.max(distanceMeters, 1);
        distanceMeters = Math.min(distanceMeters, 6);

        SmartDashboard.putNumber("HDSR/interpolatorRPM", interpolator.get(distanceMeters));
        return interpolator.get(distanceMeters);
        // if (distanceMeters > Settings.HDSR.MAX_DISTANCE_METERS || distanceMeters < Settings.HDSR.MIN_DISTANCE_METERS ) {
        //     distanceMeters = (distanceMeters > Settings.HDSR.MAX_DISTANCE_METERS ) ? Settings.HDSR.MAX_DISTANCE_METERS : Settings.HDSR.MIN_DISTANCE_METERS;  
        // }

        // // return distanceMeters*Settings.HDSR.xRPMSlope + Settings.HDSR.xRPMB;

        // Translation2d[] distanceXrpm = Settings.HDSR.distanceXrpm;
        
        // int index = 0;
        // for (int i = 0; i < distanceXrpm.length; i++) {
        //     if (distanceMeters > distanceXrpm[i].getX()) {
        //         index = i;
        //         break;
        //     }
        // }
            
        // if (distanceXrpm[index].getX() > distanceMeters) {
        //     if (index == 0) return distanceXrpm[index].getX();

        //     return distanceXrpm[index - 1].getY() + (distanceMeters - distanceXrpm[index - 1].getX()) * ((distanceXrpm[index].getY() - distanceXrpm[index - 1].getY()) / (distanceXrpm[index].getX() - distanceXrpm[index - 1].getX()));
        // }
        // else if (distanceXrpm[index].getX() == distanceMeters) {
        //     return distanceXrpm[index].getY();
        // }
        // else {
        //     return distanceXrpm[index + 1].getY() + (distanceMeters - distanceXrpm[index + 1].getX()) * ((distanceXrpm[index].getY() - distanceXrpm[index + 1].getY()) / (distanceXrpm[index].getX() - distanceXrpm[index + 1].getX()));
        // }
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

