package com.stuypulse.robot.subsystems.superStructure.arm;
import com.stuypulse.stuylib.math.SLMath;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Ports;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSim extends Arm {
    private final SingleJointedArmSim armSim;
    
    private double ArmVolts;


    public ArmSim(){
    armSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1),
        100,                                // Gear ratio
        3.0,                       // Moment of inertia (kg/m²)
        Constants.Telescope.ARM_LENGTH,             // Arm length
        Units.degreesToRadians(-90),                // Min angle
        Units.degreesToRadians(180),        // Max angle
        true,                       // Enable gravity
        Units.degreesToRadians(0)           // Start angle
    );
    }
    
    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRadians(armSim.getAngleRads());
    }
    
    public void setTargetAngle(double deg){
        armSim.setState(Units.degreesToRadians(deg), 0);
    }

    private Rotation2d getTargetAngle() {
        return Rotation2d.fromDegrees(
            SLMath.clamp(getState().getTargetAngle().getDegrees(), Settings.Arm.MIN_ANGLE.getDegrees(), Settings.Arm.MAX_ANGLE.getDegrees()));
    }
    
    private boolean isWithinTolerance(Rotation2d tolerance) {
        return Math.abs(getCurrentAngle().getDegrees() - getTargetAngle().getDegrees()) < tolerance.getDegrees();
    }

    
    @Override
    public boolean atTargetAngle(){
        return isWithinTolerance(Settings.Arm.ANGLE_TOLERANCE);
    }   


    @Override
    public void periodic(){
        simulationPeriodic();

        SmartDashboard.putNumber("Arm/Angle", getCurrentAngle().getDegrees());
        SmartDashboard.putNumber("Arm/TargetAngle", getTargetAngle().getDegrees());
        SmartDashboard.putData("Arm Sim", mArm);
    }
   
    public void simulationPeriodic(){
        armSim.setInputVoltage(ArmVolts);
        armSim.update(0.20);
    }

    Mechanism2d mArm = new Mechanism2d(90,90);
    MechanismRoot2d mArmPivot = mArm.getRoot("Pivot", 30, 30);
    MechanismLigament2d mArmTower = 
        mArmPivot.append(new MechanismLigament2d(
            "Arm", 30, 60) // thickness, color
        );

        



}