package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.constants.Settings;


public class ArmMoveToL1 extends ArmMoveToAngle {
    
    public ArmMoveToL1() {
        super(Settings.Arm.L1_ANGLE);
    }

}