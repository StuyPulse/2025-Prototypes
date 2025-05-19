package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.hooded.Turret;

import edu.wpi.first.wpilibj2.command.Command;


public class TurretDefaultCommand extends Command {
    private final Turret turret;

    public TurretDefaultCommand(){
        turret = Turret.getInstance();

        addRequirements(turret);
    }

    @Override
    public void execute() {
        // TODO add states
    }
}
