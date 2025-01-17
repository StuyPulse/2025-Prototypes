package com.stuypulse.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase{

    // Reduction between motors and encoder, as output over input. If the flywheel spins slower than
    // the motors, this number should be greater than one.
    private static final double gearing = 2.0;
    private static final double momentOfInertia = 0.0007286727441870001; // kg * m*2

    private final SparkMax motor;

    private final SmartNumber targetRPM;

    private LinearSystemLoop<N1, N1, N1> linearSystemLoop;

    // private LinearSystemSim<N1, N1, N1> flywheelSim;
    
    public Flywheel() {
        this.motor = new SparkMax(20, MotorType.kBrushed);
        SparkBaseConfig motorConfig = new SparkMaxConfig().inverted(false).idleMode(IdleMode.kCoast);
        motorConfig.encoder.positionConversionFactor(1/gearing).velocityConversionFactor(1/gearing);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.targetRPM = new SmartNumber("Flywheel/Target RPM", 0);

        LinearSystem<N1, N1, N1> flywheel = LinearSystemId.createFlywheelSystem(DCMotor.getCIM(1), momentOfInertia, gearing);
        // LinearSystem<N1, N1, N1> flywheel = LinearSystemId.identifyVelocitySystem(0.00015, 0.0);

        LinearQuadraticRegulator<N1, N1, N1> lqr = new LinearQuadraticRegulator<N1, N1, N1>(
            flywheel,
            VecBuilder.fill(4.0),
            VecBuilder.fill(12.0), 
            0.020
        );

        Matrix<N2, N1> stateSTDEVs = new Matrix<>(Nat.N2(), Nat.N1());
        stateSTDEVs.fill(10.0);
        
        Matrix<N2, N1> measurementSTDEVs = new Matrix<>(Nat.N2(), Nat.N1());
        measurementSTDEVs.fill(0.01);

        KalmanFilter<N1, N1, N1> kalmanFilter = new KalmanFilter<>(
            Nat.N1(), 
            Nat.N1(), 
            flywheel, 
            VecBuilder.fill(10.0), 
            VecBuilder.fill(0.01), 
            0.020
        );

        this.linearSystemLoop = new LinearSystemLoop<>(flywheel, lqr, kalmanFilter, 12, 0.020);

        // this.flywheelSim = new FlywheelSim(flywheel, DCMotor.getCIM(1), 0.01);
    }

    public void setTargetRPM(double RPM) {
        this.targetRPM.set(RPM);
    }

    public double getTargetRPM() {
        return targetRPM.get();
    }

    public double getRPM() {
        return motor.getEncoder().getVelocity();
        // return flywheelSim.getOutput().get(0, 0);
    }

    @Override
    public void periodic() {
        linearSystemLoop.setNextR(VecBuilder.fill(targetRPM.get()));
        linearSystemLoop.correct(VecBuilder.fill(getRPM()));
        linearSystemLoop.predict(0.020);

        double targetVoltage = linearSystemLoop.getU(0);

        motor.setVoltage(targetVoltage);
        // flywheelSim.setInput(targetVoltage);
        // flywheelSim.update(0.020);

        SmartDashboard.putNumber("Flywheel/Target Voltage", targetVoltage);
        SmartDashboard.putNumber("Flywheel/RPM", getRPM());
        SmartDashboard.putNumber("Flywheel/Controllers/kP", linearSystemLoop.getController().getK().get(0, 0));
        SmartDashboard.putNumber("Flywheel/Controllers/idk", linearSystemLoop.getFeedforward().getR().get(0, 0));
    }
}
