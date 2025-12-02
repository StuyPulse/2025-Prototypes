package com.stuypulse.robot.subsystems.HDSR;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.stuylib.network.SmartNumber;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.InterpUtil;

public class HoodedShooterImpl extends HoodedShooter {
    private final TalonFX hoodMotor;
    private final TalonFX rollerMotor;
    private final TalonFX shooterMotor;
   // private final SmartNumber setRPM = new SmartNumber("HDSR/ setRPM", getShooterState().getTargetRPM());
    private Translation2d targetTranlation;
    private final SmartNumber hoodPosition;
    private final SmartNumber shooterRPM;
    private final SmartNumber intakeRPM;

    public HoodedShooterImpl() {
        super();

        hoodMotor = new TalonFX(Ports.HDSR.HOOD_MOTOR);
        rollerMotor = new TalonFX(Ports.HDSR.ROLLER_MOTOR);
        shooterMotor = new TalonFX(Ports.HDSR.SHOOTER_MOTOR);
                
        Motors.HDSR_SHOOTER_MOTOR_CONFIG.configure(shooterMotor);
        Motors.HDSR_ROLLER_MOTOR_CONFIG.configure(rollerMotor);
        Motors.HDSR_HOOD_MOTOR_CONFIG.configure(hoodMotor);

        targetTranlation = new Translation2d();
        hoodPosition = new SmartNumber("HDSR/hood/ hood position", 0);
        intakeRPM = new SmartNumber("HDSR/Intake/ intake RPM", 0);
        shooterRPM = new SmartNumber("HDSR/Intake/ shooter RPM", 0);
        
    }

    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRotations(hoodMotor.getPosition().getValueAsDouble());
    }

    public double getCurrentVelocity() {
        return shooterMotor.getVelocity().getValueAsDouble()* 60;
    }

    public void updateTargetTranslation(Translation2d targetTranslation) {
        this.targetTranlation = targetTranslation;
    }

   
    @Override
    public void periodic() {
        super.periodic();

        hoodMotor.setControl(new PositionVoltage( hoodPosition.getAsDouble()).withSlot(0));
        shooterMotor.setControl(new VelocityVoltage(shooterRPM.getAsDouble() / 60).withSlot(0));
        rollerMotor.setControl(new VelocityVoltage(intakeRPM.getAsDouble() / 60).withSlot(0));

        //hoodMotor.setControl(new PositionVoltage(getHoodState().getTargetAngle().getRotations()).withSlot(0));
        //shooterMotor.setControl(new VelocityVoltage(getHDSRState().getShooterState().getTargetRPM() / 60.0).withSlot(0));
        SmartDashboard.putNumber("HDSR/currentVelocity", getCurrentVelocity());
        SmartDashboard.putNumber("HDSR/ Shooter target velocity ", getShooterState().getTargetRPM());
    }
}
