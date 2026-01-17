package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

public class ShooterStop extends ShooterSetState {
    public ShooterStop(){
        super(ShooterState.STOP);
    }
}
