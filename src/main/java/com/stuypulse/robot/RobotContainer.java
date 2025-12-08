/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import org.photonvision.estimation.VisionEstimation;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.hdsr.HDSRSetState;
import com.stuypulse.robot.commands.hdsr.HDSRSetShootDistance;
import com.stuypulse.robot.commands.swerve.SeedGyro;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.hdsr.HoodedShooter;
import com.stuypulse.robot.subsystems.hdsr.HoodedShooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.LimeLightVisionImpl;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    SmartNumber setdistanceToTarget = new SmartNumber("HDSR/ Settings/ SetDistanceToTarget", 4.2);

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem
    private final HoodedShooter hdsr = HoodedShooter.getInstance();
    private final SwerveDrive swerve = SwerveDrive.getInstance();
    private final LimeLightVisionImpl vison = LimelightVision.getInstance();


    // Autons
    private static SendableChooser<Command> autonChooser = new SendableChooser<>();

    // Robot container

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
        configureAutons();
    }

    /****************/
    /*** DEFAULTS ***/
    /****************/

    private void configureDefaultCommands() {
       swerve.setDefaultCommand(new SwerveDriveDrive(driver));
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        driver.getBottomButton()
            .onTrue(new HDSRSetState(ShooterState.STOW));
        driver.getRightButton()
            .onTrue(new HDSRSetState(ShooterState.SHOOTRPM));
        driver.getTopButton()
            .onTrue(new HDSRSetShootDistance(() -> setdistanceToTarget.getAsDouble()));
        driver.getRightMenuButton()
            .onTrue(new SeedGyro());

    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());
        SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
