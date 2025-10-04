/************************ PROJECT PHIL ************************/
/* Copyright (c) 2024 StuyPulse Robotics. All rights reserved.*/
/* This work is licensed under the terms of the MIT license.  */
/**************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.auton.DoNothingAuton;
// import com.stuypulse.robot.commands.differentialWrist.roller.DifferentialWristRollerStop;
import com.stuypulse.robot.commands.differentialWrist.wrist.DifferentialWristSetWristState;
import com.stuypulse.robot.commands.differentialWrist.wrist.DifferentialWristToL1;
import com.stuypulse.robot.commands.differentialWrist.wrist.DifferentialWristToL2;
import com.stuypulse.robot.commands.differentialWrist.wrist.DifferentialWristToL3;
import com.stuypulse.robot.commands.differentialWrist.wrist.DifferentialWristToStow;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.differentialWrist.DifferentialWrist;
import com.stuypulse.robot.subsystems.differentialWrist.DifferentialWrist.WristState;
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
    private final DifferentialWrist wrist = DifferentialWrist.getInstance();

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

    private void configureDefaultCommands() {}

    /***************/
    /*** BUTTONS ***/
    /***************/

    private void configureButtonBindings() {
        driver.getBottomButton().whileTrue(new DifferentialWristToL1());
        driver.getTopButton().whileTrue(new DifferentialWristToStow());
        driver.getLeftButton().whileTrue(new DifferentialWristToL2());
        driver.getRightButton().whileTrue(new DifferentialWristToL3());
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
