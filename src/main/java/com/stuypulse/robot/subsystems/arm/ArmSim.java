package com.stuypulse.robot.subsystems.arm;

import com.stuypulse.robot.constants.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSim extends Arm {
    // Physics simulations
    private final SingleJointedArmSim shoulderSim;
    private final SingleJointedArmSim elbowSim;
    
    private double prevShoulderPos = 0;
    private double prevElbowPos = 0;

    public ArmSim() {
        // Shoulder joint (with gravity)
        shoulderSim = new SingleJointedArmSim(
            DCMotor.getFalcon500(1),  // 1 motor
            100,                       // Gear ratio
            3.0,                       // Moment of inertia (kg/m²)
            Constants.SHOULDER_LENGTH,           // Arm length
            Units.degreesToRadians(-90), // Min angle
            Units.degreesToRadians(180), // Max angle
            true,                      // Enable gravity
            Units.degreesToRadians(0)   // Start angle
        );
        
        // Elbow joint (less gravity effect)
        elbowSim = new SingleJointedArmSim(
            DCMotor.getFalcon500(1),
            50,
            1.0,
            ELBOW_LENGTH,
            Units.degreesToRadians(-45),
            Units.degreesToRadians(135),
            false,
            Units.degreesToRadians(0)
        );
    }

    @Override
    public void periodic() {
        simulationPeriodic();
        
        // Logging
        SmartDashboard.putNumber("Arm/Shoulder Angle", getShoulderAngleDegrees());
        SmartDashboard.putNumber("Arm/Elbow Angle", getElbowAngleDegrees());
        SmartDashboard.putNumber("Arm/End Height", getEndPosition().getY());
    }

    @Override
    public void simulationPeriodic() {
        shoulderSim.setInputVoltage(shoulderVolts);
        elbowSim.setInputVoltage(elbowVolts);
        shoulderSim.update(0.020);
        elbowSim.update(0.020);
    }

    @Override
    public double getShoulderAngleDegrees() {
        return Units.radiansToDegrees(shoulderSim.getAngleRads());
    }

    @Override
    public double getElbowAngleDegrees() {
        return Units.radiansToDegrees(elbowSim.getAngleRads());
    }

    @Override
    public Translation2d getEndPosition() {
        double shoulderRad = Units.degreesToRadians(getShoulderAngleDegrees());
        double elbowRad = Units.degreesToRadians(getElbowAngleDegrees());
        
        return new Translation2d(
            SHOULDER_LENGTH * Math.cos(shoulderRad) + ELBOW_LENGTH * Math.cos(shoulderRad + elbowRad),
            SHOULDER_LENGTH * Math.sin(shoulderRad) + ELBOW_LENGTH * Math.sin(shoulderRad + elbowRad)
        );
    }


    @Override
    public void setTargetAngles(double shoulderDeg, double elbowDeg) {
        // Apply voltages through the control loop
        double shoulderRad = Units.degreesToRadians(shoulderDeg);
        double elbowRad = Units.degreesToRadians(elbowDeg);
        
        shoulderSim.setState(shoulderRad, 0);
        elbowSim.setState(elbowRad, 0);
    }
}