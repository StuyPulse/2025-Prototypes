package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.subsystems.spindexerNeo.SpindexerNeo;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SpindexerSim extends SpindexerNeo {
    private final DCMotorSim neoSim;
    Mechanism2d canvas;
    MechanismRoot2d spindexerArm;
    MechanismLigament2d actualArm;

    // private final PIDController controller;

    // private final ArmFeedforward ffController;


    public SpindexerSim() { //TODO: wrap, and add color for when shooter activated
        //TODO: update the gear ratio and the JKgMetersSquared !!
        neoSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.001, 40), DCMotor.getNEO(1));
        //SmartDashboard.putData("Spindexer Sim", canvas);
        canvas = new Mechanism2d(4, 4);
        spindexerArm = canvas.getRoot("SPINDEXER ARM", 2, 2);
        actualArm = spindexerArm.append(new MechanismLigament2d("ACTUAL ARM", 2, 0));
        // controller = new PIDController(
        //     10,
        //     0,
        //     0
        // );

        // ffController = new ArmFeedforward(
        //     0.0,
        //     0.0,
        //     0.0,
        //     0.0,
        //     0.020
        // );
    }

    // public Supplier<Double> getDegrees() {
    //     return () -> (Math.toDegrees(neoSim.getAngularPositionRad()) != 359 ? 
    //     SLMath.clamp(
    //         Math.toDegrees(neoSim.getAngularPositionRad()),
    //         0,
    //         360
    //     ) : 0);
    // }

    @Override
    public void periodic() {
        actualArm.setAngle(Math.toDegrees(neoSim.getAngularPositionRad()) % 360);
        SmartDashboard.putData("Spindexer Sim", canvas);

        //controller.calculate(neoSim.getAngularPositionRad(), getNeoState().getNeoSpindexerSpeed().get());
        neoSim.setInputVoltage(getNeoState().getNeoSpindexerSpeed().get() * 12);
        neoSim.update(0.02);

        SmartDashboard.putNumber("SPINDEXER SIM/ VOLTAGE", neoSim.getInputVoltage());
        SmartDashboard.putNumber("SPINDEXER SIM/ ANGULAR POS", Math.toDegrees(neoSim.getAngularPositionRad()) % 360);

    }
}
