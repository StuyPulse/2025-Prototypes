package com.stuypulse.robot.subsystems.HDSR;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;

public class HoodedShooterImpl extends HoodedShooter {
    private final TalonFX hoodMotor;
    private final TalonFX rollerMotor;
    private final TalonFX shooterMotor;
    private final SmartNumber setRPM = new SmartNumber("HDSR/ setRPM", getShooterState().getTargetRPM());
    private final InterpolatingDoubleTreeMap interpolator;
    private final Translation2d[] distanceXrpm = Settings.WiffleWorksShooter.distanceXrpm;
    private Translation2d targetTranlation;

    public HoodedShooterImpl() {
        super();

        interpolator = new InterpolatingDoubleTreeMap();
        for(Translation2d cords : distanceXrpm) {
            interpolator.put(cords.getX(), cords.getY());
        }

        hoodMotor = new TalonFX(Ports.HDSR.HOOD_MOTOR);
        rollerMotor = new TalonFX(Ports.HDSR.ROLLER_MOTOR);
        shooterMotor = new TalonFX(Ports.HDSR.SHOOTER_MOTOR, "swerve");
                
        Motors.SHOOTER_MOTOR_CONFIG.configure(shooterMotor);
        Motors.ROLLER_MOTOR_CONFIG.configure(rollerMotor);
        Motors.HOOD_MOTOR_CONFIG.configure(hoodMotor);

        targetTranlation = new Translation2d();
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(hoodMotor.getPosition().getValueAsDouble());
    }

    public double getCurrentVelocity() {
        return shooterMotor.getVelocity().getValueAsDouble()* 60;
    }

    // public void setRollerSpeeds(double speed){
    //     rollerMotor.set(speed);
    // }

    public double RPMToDistanceInterpolation() {
        double targetDistance = Odometry.getInstance().getPose().getTranslation().getDistance(targetTranlation);
        return interpolator.get(targetDistance);
        }
   
    @Override
    public void periodic() {
        super.periodic();

        hoodMotor.setControl(new PositionVoltage(getHoodState().getTargetAngle().getRotations()));
        shooterMotor.setControl(new VelocityVoltage(setRPM.getAsDouble() / 60.0).withSlot(0));
        
        SmartDashboard.putNumber("HDSR/currentVelocity", getCurrentVelocity());
        SmartDashboard.putNumber("HDSR/ Shooter target velocity ", getShooterState().getTargetRPM());
        // SmartDashboard.putNumber("HDSR/currentAngle", getCurrentAngle().getDegrees());
    }
}
