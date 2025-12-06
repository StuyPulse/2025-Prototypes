/************************ PROJECT JIM *************************/
/* Copyright (c) 2023 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot.subsystems.swerve;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Settings.Swerve.Drive;
import com.stuypulse.robot.constants.Settings.Swerve.Turn;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SimModule extends SwerveModule {

    
   
    private static LinearSystem<N2, N1, N2> identifyVelocityPositionSystem(double kV, double kA) {
        if (kV <= 0.0) {
            throw new IllegalArgumentException("Kv must be greater than zero.");
        }
        if (kA <= 0.0) {
            throw new IllegalArgumentException("Ka must be greater than zero.");


        }

         Matrix<N2, N2> systemMatrix = new Matrix<>(Nat.N2(), Nat.N2());
         systemMatrix.set(0, 0, 0.0);
         systemMatrix.set(0, 1, 1.0);
         systemMatrix.set(1, 0, 0.0);
         systemMatrix.set(1, 1, -kV / kA);

        Matrix<N2, N1> inputMatrix = new Matrix<>(Nat.N2(), Nat.N1());
        inputMatrix.set(0, 0, 0.0);
        inputMatrix.set(1, 0, 1.0 / kA);

        Matrix<N2, N2> outputMatrix = new Matrix<>(Nat.N2(), Nat.N2());
        outputMatrix.set(0, 0, 1.0);
        outputMatrix.set(0, 1, 0.0);
        outputMatrix.set(1, 0, 0.0);
        outputMatrix.set(1, 1, 1.0);

        Matrix<N2, N1> feedthroughMatrix = new Matrix<>(Nat.N2(), Nat.N1());
        feedthroughMatrix.fill(0.0);

        return new LinearSystem<N2, N1, N2>(systemMatrix, inputMatrix, outputMatrix, feedthroughMatrix);
    }

    private final LinearSystemSim<N2, N1, N2> turnSim;
    private final LinearSystemSim<N2, N1, N2> driveSim;

    // controllers
    private PIDController driveControllerPID;
    private SimpleMotorFeedforward driveFeedForward;
    private PIDController turnControllerPID;
    
    public double turnOutput;
    public double driveOutput;

    public SimModule(String name, Translation2d offset) {
        super(name, offset);

        this.driveFeedForward = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);
        this.driveControllerPID = new PIDController(Drive.kP, Drive.kI, Drive.kD);
        this.turnControllerPID = new PIDController(Turn.kP, Turn.kI, Turn.kD);

        turnSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(Turn.kV, Turn.kA));
        driveSim = new LinearSystemSim<>(identifyVelocityPositionSystem(Drive.kV, Drive.kA));
    }
    //private Encoder m_rightEncoder = new Encoder(Ports.Swerve.FrontRight.ENCODER, Ports.Swerve.BackRight.ENCODER);
    //private Encoder m_leftEncoder = new Encoder(Ports.Swerve.FrontLeft.ENCODER, Ports.Swerve.BackLeft.ENCODER);

    // private EncoderSim m_leftSimEncoder; 
    // private EncoderSim m_rightSimEncoder; 

    // public void DrivertainSim() {
    //     //super();
    //     //m_leftSimEncoder = new EncoderSim(m_leftEncoder);
    //     //m_rightSimEncoder = new EncoderSim(m_rightEncoder);

    //     m_leftSimEncoder.setDistancePerPulse(2f * Math.PI * 3f / 8400f);
    //     m_rightSimEncoder.setDistancePerPulse(2f * Math.PI * 3f / 8400f);
    // }
    

    @Override
    public double getVelocity() {
        return driveSim.getOutput(1);
    }

    private double getDistance() {
        return driveSim.getOutput(0);
    }

    @Override
    public double getTargetRPM() {
        return 0.0;
    }

    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(turnSim.getOutput(0));
    }

    
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDistance(), getAngle());
    }

    public void updateTurnContoller() {
        this.turnOutput = turnControllerPID.calculate(getModulePosition().angle.getRadians(), getAngle().getRadians());
    }
    public void updateDrive() {
        this.driveOutput = driveControllerPID.calculate(getVelocity(), getModulePosition().distanceMeters) + driveFeedForward.calculate(getVelocity());
    }
    
    
    @Override
    public void periodic() {
        super.periodic();

        updateTurnContoller();
        updateDrive();
        //calculate PID for turn
        //calculate PID for drive
        //FF and PID CONTROLLER SUM
       
//         turnController.update(
//             Angle.fromRotation2d(getTargetState().angle),
//             Angle.fromRotation2d(getAngle()));
// //
//         driveController.update(
//             getTargetState().speedMetersPerSecond,
//             getVelocity());

        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Sim Angle Voltage", turnOutput);
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Sim Velocity Voltage", driveOutput);
    }

    @Override
    public void simulationPeriodic() {
        // drive
        updateDrive();
        driveSim.setInput(driveOutput);
        driveSim.update(Settings.DT);

        // turn
        updateTurnContoller();
        turnSim.setInput(turnOutput);
        turnSim.update(Settings.DT);

        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Sim Angle Voltage", turnOutput);
        SmartDashboard.putNumber("Swerve/Modules/" + getName() + "/Sim Velocity Voltage", driveOutput);
    }
}