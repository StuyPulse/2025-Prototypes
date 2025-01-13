package com.stuypulse.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

public class ElevatorVisualizer {

    private final Mechanism2d elevator2d;
    private final ElevatorImpl elevator;

    public ElevatorVisualizer(ElevatorImpl elevator) {
        this.elevator = elevator;
         
        elevator2d = new Mechanism2d(6, 15);
        
        // Elevator Frame
        elevator2d.getRoot("Base Origin", 0, 0).append(new MechanismLigament2d(
            "Left Tower",
            14, 
            0,
            10, 
            new Color8Bit(Color.kOrange)));

        elevator2d.getRoot("Base Origin", 0, 0).append(new MechanismLigament2d(
            "Bottom Tower",
            6, 
            -90,
            10, 
            new Color8Bit(Color.kOrange)));

        elevator2d.getRoot("Base Origin", 6, 0).append(new MechanismLigament2d(
            "Right Tower", 
            14,
            0,
            10,
            new Color8Bit(Color.kOrange)
        ));

        elevator2d.getRoot("Base Origin", 0, 14).append(new MechanismLigament2d(
            "Top Side",
            6,
            -90, 
            10, 
            new Color8Bit(Color.kOrange)));


        //outerFrame
        elevator2d.getRoot("Base Origin", 1, 1).append(new MechanismLigament2d(
            "Left Side",
            14,
            0, 
            10,
            new Color8Bit(Color.kYellow)));
    
        elevator2d.getRoot("Base Origin", 1, 1).append(new MechanismLigament2d(
            "Bottom Side",
            4,
            -90,
            10,
            new Color8Bit(Color.kYellow)));
            
        elevator2d.getRoot("Base Origin", 1, 15).append(new MechanismLigament2d(
            "Top Side",
            4,
            -90,
            10,
            new Color8Bit(Color.kYellow)));

        elevator2d.getRoot("Base Origin", 5, 1).append(new MechanismLigament2d(
            "Right Side",
            14,
            0,
            10,
            new Color8Bit(Color.kYellow)));
        

        //innerFrame
        elevator2d.getRoot("Base Origin", 2, 2).append(new MechanismLigament2d(
            "Bottom Side",
            2,
            -90, 
            10,
            new Color8Bit(Color.kPink)));

        elevator2d.getRoot("Base Origin", 2, 2).append(new MechanismLigament2d(
            "Left Side",
            2,
            0, 
            10,
            new Color8Bit(Color.kPink)));
            
        elevator2d.getRoot("Base Origin", 2, 4).append(new MechanismLigament2d(
            "Top Side",
            2,
            -90, 
            10,
            new Color8Bit(Color.kPink)));
            
        elevator2d.getRoot("Base Origin", 4, 2).append(new MechanismLigament2d(
            "Right Side",
            2,
            0, 
            10,
            new Color8Bit(Color.kPink)));
    }

    public void update() {

    }
}