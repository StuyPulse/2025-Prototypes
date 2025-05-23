package com.stuypulse.robot.commands.turret;

import com.stuypulse.robot.subsystems.hooded.Turret;
import com.stuypulse.robot.subsystems.hooded.Turret.TurretState;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class TurretToAngle extends Command {
    
    private final Turret turret;
    private final Rotation2d angle;

    public TurretToAngle(Rotation2d angle) {
        turret = Turret.getInstance();
        this.angle = angle;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setState(TurretState.FREE);
        turret.setTargetAngle(angle);
    }

}
