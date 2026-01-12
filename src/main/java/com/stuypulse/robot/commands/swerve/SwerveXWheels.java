package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.subsystems.swerve.SwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;


public class SwerveXWheels extends Command {
    private final SwerveDrive swerve;

    public SwerveXWheels() {
        swerve = SwerveDrive.getInstance();

        addRequirements(swerve);
    }

    @Override 
    public void execute() {
        swerve.setXMode();
    }
}
