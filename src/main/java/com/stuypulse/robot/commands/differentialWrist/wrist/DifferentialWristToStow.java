package com.stuypulse.robot.commands.differentialWrist.wrist;

import com.stuypulse.robot.subsystems.differentialWrist.DifferentialWrist.WristState;

public class DifferentialWristToStow extends DifferentialWristSetWristState {
    public DifferentialWristToStow() {
        super(WristState.STOW);
    }
}
