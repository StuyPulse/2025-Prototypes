package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterSetShoot extends Command {
    private Shooter shooter;

    public ShooterSetShoot() {
        shooter = Shooter.getInstance();
        shooter.setShooterState(ShooterState.SHOOT);

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterState(ShooterState.SHOOT);
    }
}
