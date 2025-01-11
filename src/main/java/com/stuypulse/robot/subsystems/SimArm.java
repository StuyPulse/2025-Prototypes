;package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimArm extends Arm {
    private final SingleJointedArmSim sim;

    private final PIDController controller;

    private final SmartNumber targetAngle;

    protected SimArm() {
        sim = new SingleJointedArmSim(
            DCMotor.getNEO(1), 
            Settings.Arm.GEAR_RATIO, 
            Settings.Arm.MOMENT_OF_INERTIA, 
            Settings.Arm.ARM_LENGTH, 
            Settings.Arm.LOWER_ANGLE_LIMIT, 
            Settings.Arm.UPPER_ANGLE_LIMIT, 
            false, 
            Settings.Arm.LOWER_ANGLE_LIMIT, 
            null);
        
        controller = new PIDController(Settings.Arm.PID.KP, Settings.Arm.PID.KI, Settings.Arm.PID.KD);

        targetAngle = new SmartNumber("Arm/Target Angle", 0.0);
    }

    @Override
    public double getAngle() {
        return super.getAngle();
    }

    @Override
    public void setState(State state) {

    }
    
    // @Override
    // public void setL1() {
    //     super.setL1();
    // }

    // @Override
    // public void setL2() {
    //     super.setL2();
    // }

    // @Override
    // public void setL3() {
    //     super.setL3();
    // }

    // @Override
    // public void setL4() {
    //     super.setL4();
    // }
}
