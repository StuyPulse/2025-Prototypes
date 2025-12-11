package com.stuypulse.robot.subsystems.hdsr;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.stuypulse.robot.constants.Motors;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.odometry.Odometry;
import com.stuypulse.robot.util.InterpUtil;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HoodedShooterImpl extends HoodedShooter {
    private final TalonFX shooterMotor;
    private final SparkMax rollerMotor;
    private final RelativeEncoder rollerEncoder;
    private final PIDController rollerController;
    private final SimpleMotorFeedforward rollerFFController;

    private Translation2d[] distancexRPM;
    private InterpolatingDoubleTreeMap interpolator; 
    private Translation2d targetTranslation;


    private final Odometry odometry;

    private SmartNumber setRPMHDSR;
    private SmartNumber setRollerRPM;

    public HoodedShooterImpl() {
        super();
        
        interpolator = new InterpolatingDoubleTreeMap();
        distancexRPM = Settings.HDSR.levelDistanceXrpm;
        for (Translation2d point : distancexRPM) {
            interpolator.put(point.getX(), point.getY());
        }

        shooterMotor = new TalonFX(Ports.HDSR.SHOOTER_MOTOR, "swerve");
        Motors.SHOOTER_MOTOR_CONFIG.configure(shooterMotor);

        rollerMotor = new SparkMax(Ports.HDSR.ROLLER_MOTOR, MotorType.kBrushless);
        rollerMotor.configure(Motors.HoodedShooter.motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rollerEncoder = rollerMotor.getEncoder();

        odometry = Odometry.getInstance();

        setRPMHDSR = new SmartNumber("HDSR/Settings/set Shooter RPM", getShooterState().getShooterRPM());
        setRollerRPM = new SmartNumber("HDSR/Settings/set Roller RPM", getRollerState().getRollerRPM());

        targetTranslation = new Translation2d();

        rollerController = new PIDController(1.9, 0.0, 0.03);
        rollerFFController = new SimpleMotorFeedforward(0, 0.117, 0.2);
    }


    /**
     * Method to get shooter velocity in RPM
     *@return Shooter velocity in RPM 
     */
    public double getCurrentShooterVelocity() {
        return shooterMotor.getVelocity().getValueAsDouble()* 60;
    }

    public double getCurrentRollerVelocity() {
        return rollerEncoder.getVelocity();
    }

    public void setTargetTranslation(Translation2d targetTranslation) {
        this.targetTranslation = targetTranslation;
    }

    /**
     * finds the rpm to shoot to a target x meters away on a level plane
     * @return rpm needed to reach target 
     */
    public double getShootDistanceRPM() {
        Translation2d currentTranslation = odometry.getPose().getTranslation();
        double distanceToTarget = currentTranslation.getDistance(targetTranslation);

        //clamping
        distanceToTarget = Math.max(distanceToTarget, Settings.HDSR.MIN_DISTANCE_METERS);
        distanceToTarget = Math.min(distanceToTarget, Settings.HDSR.MAX_DISTANCE_METERS);

        return InterpUtil.getLevelDistanceInterp(currentTranslation);
    } 


    @Override 
    public double getShootGoalRPM() {
        return InterpUtil.getGoalDistanceInterp();
    }

    
   
    @Override
    public void periodic() {
        super.periodic();

        switch (getShooterState()) {
            case SHOOTRPM:
                    getShooterState().setShooterRPM(setRPMHDSR.getAsDouble());
                    getRollerState().setRollerRPM(setRollerRPM.getAsDouble());
                break;
            case LEVELINTERP:
                getShooterState().setShooterRPM(getShootDistanceRPM());
                break;
            case GOALINTERP:
                getShooterState().setShooterRPM(getShootGoalRPM());
                break;
            default:
                getShooterState().setShooterRPM(0.0);
                getRollerState().setRollerRPM(0.0);
                break;
        } 

        rollerController.update(getCurrentRollerVelocity(), getRollerState().getRollerRPM());
        shooterMotor.setControl(new VelocityVoltage(getShooterState().getShooterRPM() / 60.0).withSlot(0));
        rollerMotor.setVoltage(rollerController.getOutput() + rollerFFController.calculate(getRollerState().getRollerRPM()));
        
        SmartDashboard.putNumber("HDSR/Current Shooter Velocity", getCurrentShooterVelocity());
        SmartDashboard.putNumber("HDSR/Current Roller Velocity", getCurrentRollerVelocity());
        SmartDashboard.putNumber("HDSR/Target velocity ", getShooterState().getShooterRPM());
        SmartDashboard.putNumber("HDSR/interpolatorRPM", getShootDistanceRPM());

    }
}

