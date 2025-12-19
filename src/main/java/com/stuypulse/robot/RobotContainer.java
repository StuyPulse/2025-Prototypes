/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import org.photonvision.estimation.VisionEstimation;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.hdsr.HDSRSetState;
import com.stuypulse.robot.commands.hdsr.HDSRSetRollerState;
import com.stuypulse.robot.commands.hdsr.HDSRSetShootDistance;
import com.stuypulse.robot.commands.hdsr.HDSRSetShooterState;
import com.stuypulse.robot.commands.swerve.SeedGyro;
import com.stuypulse.robot.commands.swerve.SwerveDriveDrive;
import com.stuypulse.robot.commands.swerve.SwervePIDToPose;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.hdsr.HoodedShooter;
import com.stuypulse.robot.subsystems.hdsr.HoodedShooter.RollerState;
import com.stuypulse.robot.subsystems.hdsr.HoodedShooter.ShooterState;
import com.stuypulse.robot.subsystems.swerve.SwerveDrive;
import com.stuypulse.robot.subsystems.vision.LimeLightVisionImpl;
import com.stuypulse.robot.subsystems.vision.LimelightVision;
import com.stuypulse.robot.util.FieldUtil;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

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
        driver.getBottomButton() // Stow Shooter
            .onTrue(new HDSRSetState(ShooterState.STOW));
        driver.getRightButton() // Shoot RPM via SmartNumber
            .onTrue(new HDSRSetState(ShooterState.SHOOTRPM));
        driver.getTopButton() // interpolate to goal without moving
            .onTrue(new HDSRSetState(ShooterState.GOALINTERP));
        driver.getRightMenuButton() // Makes current robot front field front 
            .onTrue(new SeedGyro());
        driver.getLeftButton() // pid to pose to origin
           .onTrue(new SwervePIDToPose(new Pose2d(), operator));
        driver.getRightTriggerButton() // pid to optimal shoot pose then shoot
            .onTrue(new SequentialCommandGroup(
                        new SwervePIDToPose(() -> FieldUtil.getShootPose(), driver),
                        new HDSRSetShooterState(ShooterState.GOALINTERP)
                     ));
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
