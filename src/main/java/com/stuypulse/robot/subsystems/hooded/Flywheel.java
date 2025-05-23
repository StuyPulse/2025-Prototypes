package com.stuypulse.robot.subsystems.hooded;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stuypulse.robot.Robot;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Flywheel extends SubsystemBase{

    public static final Flywheel instance;
    
    static {
        instance = new Flywheel();
    }

    public static Flywheel getInstance(){
        return instance;
    }

    // Reduction between motors and encoder, as output over input. If the flywheel spins slower than
    // the motors, this number should be greater than one.
    private static final double gearing = 2.0;
    private static final double momentOfInertia = 0.0007915903; // kg * m*2

    private final SparkMax motor;

    private final SmartNumber targetRadPerS;

    private LinearSystemLoop<N1, N1, N1> linearSystemLoop;

    private FlywheelSim flywheelSim;
    
    public Flywheel() {
        this.motor = new SparkMax(20, MotorType.kBrushed);
        SparkBaseConfig motorConfig = new SparkMaxConfig().inverted(false).idleMode(IdleMode.kCoast);
        motorConfig.encoder.positionConversionFactor(1/gearing).velocityConversionFactor(1/gearing);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.targetRadPerS = new SmartNumber("Flywheel/Target Rad per S", 0);

        LinearSystem<N1, N1, N1> flywheel = LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), momentOfInertia, gearing);

        this.flywheelSim = new FlywheelSim(flywheel, DCMotor.getNEO(1), 2.0);

        LinearQuadraticRegulator<N1, N1, N1> lqr = new LinearQuadraticRegulator<N1, N1, N1>(
            flywheel,
            VecBuilder.fill(100.0),
            VecBuilder.fill(1.0), 
            0.020
        );

        KalmanFilter<N1, N1, N1> kalmanFilter = new KalmanFilter<>(
            Nat.N1(), 
            Nat.N1(), 
            flywheel, 
            VecBuilder.fill(10.0), 
            VecBuilder.fill(2.0), 
            0.020
        );

        this.linearSystemLoop = new LinearSystemLoop<>(flywheel, lqr, kalmanFilter, 12, 0.020);
    }

    public void setTargetRadPerS(double RPM) {
        this.targetRadPerS.set(RPM);
    }

    public double getTargetRadPerS() {
        return targetRadPerS.get();
    }

    public double getRadPerS() {
        return motor.getEncoder().getVelocity() / 60 * 2 * Math.PI;
        // return flywheelSim.getAngularVelocityRadPerSec();
    }

    @Override
    public void periodic() {
        linearSystemLoop.setNextR(VecBuilder.fill(targetRadPerS.get()));
        linearSystemLoop.correct(VecBuilder.fill(getRadPerS()));
        linearSystemLoop.predict(0.020);

        double targetVoltage = linearSystemLoop.getU(0);

        motor.setVoltage(targetVoltage);
        flywheelSim.setInputVoltage(targetVoltage);
        flywheelSim.update(0.020);

        SmartDashboard.putNumber("Flywheel/Target Voltage", targetVoltage);
        SmartDashboard.putNumber("Flywheel/Rad Per S", getRadPerS());
    }
}
