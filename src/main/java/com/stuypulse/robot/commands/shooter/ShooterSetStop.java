package com.stuypulse.robot.commands.shooter;

import com.stuypulse.robot.subsystems.shooter.Shooter;
import com.stuypulse.robot.subsystems.shooter.Shooter.ShooterState;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterSetStop extends Command {
    private Shooter shooter;

    public ShooterSetStop() {
        shooter = Shooter.getInstance();
        shooter.setShooterState(ShooterState.STOP);

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        System.out.println("STOP KILLING ALWIN");
        shooter.setShooterState(ShooterState.STOP);
    }

}
