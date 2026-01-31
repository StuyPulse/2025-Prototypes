/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;


import com.stuypulse.robot.commands.spindexer.SpindexerKrakenSetStateSpin;
import com.stuypulse.robot.commands.spindexer.SpindexerKrakenSetStateStop;
import com.stuypulse.robot.commands.spindexer.SpindexerNeoSetStateSpin;
import com.stuypulse.robot.commands.spindexer.SpindexerNeoSetStateStop;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.spindexerKraken.SpindexerKraken;
import com.stuypulse.robot.subsystems.spindexerNeo.SpindexerNeo;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    // Gamepads
    public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
    public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);
    
    // Subsystem

    private final SpindexerKraken spindexerKraken = SpindexerKraken.getInstance();
    private final SpindexerNeo spindexerNeo = SpindexerNeo.getInstance();

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
        // shooter button bindings
        // driver.getTopButton()
        //     .whileTrue(new ShooterShoot())
        //     .whileFalse(new ShooterStop());

       // driver.getTopButton().onTrue(new ShooterShoot());
        driver.getTopButton()
            .whileTrue(new SpindexerKrakenSetStateSpin())
            .whileFalse(new SpindexerKrakenSetStateStop());

       driver.getBottomButton()
            .whileTrue(new SpindexerNeoSetStateSpin())
            .whileFalse(new SpindexerNeoSetStateStop());
    }

    /**************/
    /*** AUTONS ***/
    /**************/

    public void configureAutons() {
        // autonChooser.setDefaultOption("Do Nothing", new DoNothingAuton());

        // SmartDashboard.putData("Autonomous", autonChooser);
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}
