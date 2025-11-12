package com.stuypulse.robot.commands.swerve;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.stuypulse.robot.Robot;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveDrive extends Command{
    private com.stuypulse.robot.subsystems.swerve.SwerveDrive swerve;
    private Gamepad driver;

    private final Supplier<Translation2d> speed;
    private final DoubleSupplier turn;

    public SwerveDriveDrive(Gamepad driver) {
        swerve = SwerveDrive.getInstance();
        this.driver = driver;

        speed = () -> {
            Translation2d inputVel = getInputVelocity();
            if (inputVel.getNorm() < 0.05){
                return new Translation2d();
            }
            double x = inputVel.getX();
            double y = inputVel.getY();


            x /= inputVel.getNorm();
            y /= inputVel.getNorm();

            x *= Math.abs(x) * 1.5;
            y *= Math.abs(y) * 1.5;
            
            if (Robot.isBlue()) {
                return new Translation2d(x, -y);
            } else {
                return new Translation2d(-x, y);
            }

            
        };

        
        turn = () -> {
            double inputTurn = -driver.getRightX();

            if (Math.abs(inputTurn) < 0.05){
                return 0;
            }
            
            inputTurn *= Math.signum(inputTurn) * inputTurn;

            inputTurn *= 2 * Math.PI;

            return inputTurn;
        };
        
        addRequirements(swerve);
    }

    private Translation2d getInputVelocity() {
        return new Translation2d(driver.getLeftY(), -driver.getLeftX());
    }

    @Override
    public void execute() {
        swerve.drive(speed.get(), turn.getAsDouble());
    }
}
