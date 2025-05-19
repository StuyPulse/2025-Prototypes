package com.stuypulse.robot.subsystems.superStructure.arm;

import com.stuypulse.robot.constants.Constants;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmSim extends Arm {
    private final SingleJointedArmSim sim;
    
    protected ArmSim() {
        //TBD
    }

    @Override
    public Rotation2d getCurrentAngle() {
        return Rotation2d.fromRadians(sim.getAngleRads());
    }

    @Override
    public boolean atTargetAngle() {
        return isWithinTolerance(Settings.Arm.ANGLE_TOLERANCE);
    }

}