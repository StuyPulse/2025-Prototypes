/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
import com.stuypulse.robot.commands.spindexer.SpindexerRunAll;
import com.stuypulse.robot.commands.spindexer.SpindexerRunRoller;
import com.stuypulse.robot.commands.spindexer.SpindexerRunSpinner;
import com.stuypulse.robot.commands.spindexer.SpindexerStop;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.spindexer.Spindexer;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem
    private final Spindexer spindexer = Spindexer.getInstance();

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
    }

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {

        driver.getTopButton()
            .whileTrue(new SpindexerRunAll())
            .onFalse(new SpindexerStop());

        driver.getLeftButton()
            .whileTrue(new SpindexerRunRoller())
            .onFalse(new SpindexerStop());
            
        driver.getRightButton()
            .whileTrue(new SpindexerRunSpinner())
            .onFalse(new SpindexerStop());

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
