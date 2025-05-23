package com.stuypulse.robot.commands.hooded.turret;

import com.stuypulse.robot.subsystems.hooded.Turret;
import com.stuypulse.robot.subsystems.hooded.Turret.TurretState;

import edu.wpi.first.wpilibj2.command.Command;


public class TurretToState extends Command {
    
    private final Turret turret;
    private final TurretState state;

    public TurretToState(TurretState state) {
        turret = Turret.getInstance();
        this.state = state;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setState(state);
    }

}
