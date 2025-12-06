package com.stuypulse.robot.commands.swerve;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SeedGyro extends InstantCommand{
    private final SwerveDrive drive;

    public SeedGyro() {
        drive = SwerveDrive.getInstance();

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.setgyro();
    }

}
