package com.stuypulse.robot.subsystems;

import java.util.Optional;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.SLMath;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator {

    private final ElevatorSim sim;
    private final SmartNumber targetHeight;
    private final double minHeight, maxHeight;
    private final SmartNumber maxAccel, maxVel;
    private Optional<Double> voltageOverride;
    private final Controller controller;
    
    Elevator() {
        targetHeight = new SmartNumber("Elevator/Target Height", 0);
        minHeight = Settings.Elevator.MIN_HEIGHT;
        maxHeight = Settings.Elevator.MAX_HEIGHT;
        maxAccel = new SmartNumber("Elevator/Max Acceleration",Settings.Elevator.MAX_ACCELERATION);
        maxVel = new SmartNumber("Elevator/Max Velocity", Settings.Elevator.MAX_VELOCITY);

        sim = new ElevatorSim(
            0, 
            0, 
            null, 
            0,
            0, 
            true, 
            0, 
            null);

        controller = new SimpleMotorFeedforward(Settings.Feedforward.kS, Settings.Feedforward.kV, Settings.Feedforward.kA);
            


        voltageOverride = Optional.empty();
    }

    public void setStartHeight(double height) {
        targetHeight.set(SLMath.clamp(height, minHeight, maxHeight));
        voltageOverride = Optional.empty();
    }

    public boolean elevatorBottom() {
        return sim.hasHitLowerLimit();
    }

    public boolean elevatorTop() {
        return sim.hasHitUpperLimit();
    }

    public double getHeight() {
        return sim.getPositionMeters();
    }

    public void stopElevator() {
        sim.setInputVoltage(0.0);
    }

    public void elevatorIdleMode(IdleMode mode) {}

    public void setVoltageOverride(double voltage) {
        voltageOverride = Optional.of(voltage);
    }

    public void setConstraints(double maxVelocity, double maxAcceleration) {
        this.maxVel.set(maxVelocity);
        this.maxAccel.set(maxAcceleration);
    }

    public void calculateVoltage() {
        
    }
    
    public void periodic() {
        double voltage = voltageOverride.orElse();

        if (elevatorBottom() && voltage < 0 || elevatorTop() && voltage > 0) {
            stopElevator();
        } else {
            sim.setInputVoltage(voltage);
        }
        
        SmartDashboard.putNumber("Elevator/Target Height", maxHeight);
        SmartDashboard.putNumber("Elevator/Current", sim.getCurrentDrawAmps());
        SmartDashboard.putNumber("Elevator/Height", getHeight());
    }

    public void simulationPeriodic() {
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(sim.getCurrentDrawAmps()));

        sim.update(Settings.DT);
    }

}
