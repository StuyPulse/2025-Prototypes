package com.stuypulse.robot.subsystems.wiffleWorksShooter;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.robot.constants.Settings;

public class WiffleWorksShooterImpl extends WiffleWorksShooter {
    private final TalonFX shooterMotor;
    double setRPMvalue = 3000;
    private SmartNumber setRPM = new SmartNumber("HDSR/ setRPM", getState().getTargetRPM());
    private boolean hasBall;

    public WiffleWorksShooterImpl() {
        super();
        shooterMotor = new TalonFX(Ports.HDSR.SHOOTER_MOTOR, "swerve");
        
        hasBall = false;
        
        Motors.SHOOTER_MOTOR_CONFIG.configure(shooterMotor);

    }

    public double getCurrentVelocity() {
        return shooterMotor.getVelocity().getValueAsDouble()* 60;
    }

    public double RPMToDistanceInterpolation(double distanceMeters) {
        if (distanceMeters > Settings.WiffleWorksShooter.MAX_DISTANCE_METERS || distanceMeters < Settings.WiffleWorksShooter.MIN_DISTANCE_METERS ) {
            distanceMeters = (distanceMeters > Settings.WiffleWorksShooter.MAX_DISTANCE_METERS ) ? Settings.WiffleWorksShooter.MAX_DISTANCE_METERS : Settings.WiffleWorksShooter.MIN_DISTANCE_METERS;  
        }

        // return distanceMeters*Settings.HDSR.xRPMSlope + Settings.HDSR.xRPMB;

        Translation2d[] distanceXrpm = Settings.WiffleWorksShooter.distanceXrpm;
        
        int index = 0;
        for (int i = 0; i < distanceXrpm.length; i++) {
            if (distanceMeters > distanceXrpm[i].getX()) {
                index = i;
                break;
            }
        }
            
        if (distanceXrpm[index].getX() > distanceMeters) {
            if (index == 0) return distanceXrpm[index].getX();

            return distanceXrpm[index - 1].getY() + (distanceMeters - distanceXrpm[index - 1].getX()) * ((distanceXrpm[index].getY() - distanceXrpm[index - 1].getY()) / (distanceXrpm[index].getX() - distanceXrpm[index - 1].getX()));
        }
        else if (distanceXrpm[index].getX() == distanceMeters) {
            return distanceXrpm[index].getY();
        }
        else {
            return distanceXrpm[index + 1].getY() + (distanceMeters - distanceXrpm[index + 1].getX()) * ((distanceXrpm[index].getY() - distanceXrpm[index + 1].getY()) / (distanceXrpm[index].getX() - distanceXrpm[index + 1].getX()));
        }
    }
   
    @Override
    public void periodic() {
        super.periodic();

        shooterMotor.setControl(new VelocityVoltage(setRPM.getAsDouble() / 60.0).withSlot(0));
        
        SmartDashboard.putNumber("HDSR/currentVelocity", getCurrentVelocity());
        SmartDashboard.putNumber("HDSR/ target velocity ", getState().getTargetRPM());
    }
}
