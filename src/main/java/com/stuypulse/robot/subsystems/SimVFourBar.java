package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimVFourBar extends VFourBar {
    private final SingleJointedArmSim sim;

    private final PIDController controller;

    private final SmartNumber targetAngle;

    protected SimVFourBar() {
        sim = new SingleJointedArmSim(
            DCMotor.getNEO(1), 
            Settings.VFourBar.GEAR_RATIO, 
            Settings.VFourBar.MOMENT_OF_INERTIA, 
            Settings.VFourBar.ARM_LENGTH, 
            Settings.VFourBar.LOWER_ANGLE_LIMIT, 
            Settings.VFourBar.UPPER_ANGLE_LIMIT, 
            false, 
            Settings.VFourBar.LOWER_ANGLE_LIMIT, 
            null);
        
        controller = new PIDController(Settings.VFourBar.PID.KP, Settings.VFourBar.PID.KI, Settings.VFourBar.PID.KD);

        targetAngle = new SmartNumber("VFourBar/Target Angle", 0.0);
    }

    @Override
    public double getAngle() {
        return super.getAngle();
    }

    @Override
    public void setAngle(double angle) {
        super.setAngle(angle);
    }
    
    @Override
    public void setL1() {
        super.setL1();
    }

    @Override
    public void setL2() {
        super.setL2();
    }

    @Override
    public void setL3() {
        super.setL3();
    }

    @Override
    public void setL4() {
        super.setL4();
    }
}