package com.stuypulse.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmSim extends Arm {
    // Physics simulations
    private final SingleJointedArmSim shoulderSim;
    private final SingleJointedArmSim elbowSim;
    
    // Simulated motor inputs (volts)
    private double shoulderVolts;
    private double elbowVolts;
    
    // Arm geometry (must match hardware impl)
    private final double SHOULDER_LENGTH = 0.5; // meters
    private final double ELBOW_LENGTH = 0.4;    // meters
    private final double BASE_HEIGHT = 0.2;      // meters

    public ArmSim() {
        // Shoulder joint (with gravity)
        shoulderSim = new SingleJointedArmSim(
            DCMotor.getFalcon500(1),  // 1 motor
            100,                       // Gear ratio
            3.0,                       // Moment of inertia (kg/m²)
            SHOULDER_LENGTH,           // Arm length
            Units.degreesToRadians(-90), // Min angle (deg→rad)
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
        // Update physics first
        simulationPeriodic();
        
        // Log all values in degrees
        SmartDashboard.putNumber("Arm/Shoulder Angle (deg)", getShoulderAngleDegrees());
        SmartDashboard.putNumber("Arm/Elbow Angle (deg)", getElbowAngleDegrees());
        SmartDashboard.putNumber("Arm/End Height (m)", getEndHeight());
    }

    @Override
    public void simulationPeriodic() {
        shoulderSim.setInputVoltage(shoulderVolts);
        elbowSim.setInputVoltage(elbowVolts);
        shoulderSim.update(0.020);
        elbowSim.update(0.020);
    }

    //// DEGREE-BASED INTERFACE ////
    public double getShoulderAngleDegrees() {
        return Units.radiansToDegrees(shoulderSim.getAngleRads());
    }

    public double getElbowAngleDegrees() {
        return Units.radiansToDegrees(elbowSim.getAngleRads());
    }

    public double getShoulderVelocityDegS() {
        return Units.radiansToDegrees(shoulderSim.getVelocityRadPerSec());
    }

    public double getElbowVelocityDegS() {
        return Units.radiansToDegrees(elbowSim.getVelocityRadPerSec());
    }

    //// CONTROL ////
    @Override
    public void setVoltages(double shoulderVolts, double elbowVolts) {
        this.shoulderVolts = shoulderVolts;
        this.elbowVolts = elbowVolts;
    }

    //// KINEMATICS ////
    public Translation2d getEndPosition() {
        double shoulderRad = Units.degreesToRadians(getShoulderAngleDegrees());
        double elbowRad = Units.degreesToRadians(getElbowAngleDegrees());
        
        double x = SHOULDER_LENGTH * Math.cos(shoulderRad) 
                 + ELBOW_LENGTH * Math.cos(shoulderRad + elbowRad);
        double y = SHOULDER_LENGTH * Math.sin(shoulderRad) 
                 + ELBOW_LENGTH * Math.sin(shoulderRad + elbowRad);
        
        return new Translation2d(x, y);
    }

    public double getEndHeight() {
        return BASE_HEIGHT + getEndPosition().getY();
    }

    @Override
    public void stop() {
        setVoltages(0, 0);
    }

    @Override
    public void resetEncoders() {
        shoulderSim.setState(0, 0);
        elbowSim.setState(0, 0);
    }
}