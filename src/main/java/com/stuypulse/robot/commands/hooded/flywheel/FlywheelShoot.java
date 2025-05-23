package com.stuypulse.robot.commands.hooded.flywheel;

import com.stuypulse.robot.subsystems.hooded.flywheel.Flywheel;

import edu.wpi.first.wpilibj2.command.Command;

public class FlywheelShoot extends Command{
    
    private final Flywheel flywheel;
    private final double radPerS;

    public FlywheelShoot(double velocity) {
        flywheel = Flywheel.getInstance();
        radPerS = velocity;
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        flywheel.setTargetRadPerS(radPerS);
    }

}
