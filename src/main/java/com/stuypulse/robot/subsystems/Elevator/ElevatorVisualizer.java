package com.stuypulse.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;

public class ElevatorVisualizer {

    private final Mechanism2d elevator2d;
    private final ElevatorImpl elevator;

    private final MechanismRoot2d elevatorBL;
    private final MechanismRoot2d elevatorTR;

    private final MechanismRoot2d outerBL;
    private final MechanismRoot2d outerTR;

    private final MechanismRoot2d innerBL;
    private final MechanismRoot2d  innerTR;



    public ElevatorVisualizer(ElevatorImpl elevator) {

        // Mechanism 
        this.elevator = elevator;
        
        elevator2d = new Mechanism2d(6, 15);
        
        // Elevator Frame

        // bottom left node
        elevatorBL = elevator2d.getRoot("Elevator BL", 0, 0);

        elevatorBL.append(new MechanismLigament2d(
            "Left Tower",
            14, 
            0,
            10, 
            new Color8Bit(Color.kOrange)
        ));

        elevatorBL.append(new MechanismLigament2d(
            "Bottom Tower",
            6, 
            -90,
            10, 
            new Color8Bit(Color.kOrange)
        ));

        // top right node
        elevatorTR = elevator2d.getRoot("Elevator TR", 6, 0);

        elevatorTR.append(new MechanismLigament2d(
            "Right Tower", 
            14,
            180,
            10,
            new Color8Bit(Color.kOrange)
        ));

        elevatorTR.append(new MechanismLigament2d(
            "Top Side",
            6,
            90, 
            10, 
            new Color8Bit(Color.kOrange)));

        //outerFrame

        // bottom left node
        outerBL = new 
        elevator2d.getRoot("Outer Bottom Left", 1, 1).append(new MechanismLigament2d(
            "Left Side",
            14,
            0, 
            10,
            new Color8Bit(Color.kYellow)));
    
        elevator2d.getRoot("Outer Bottom Left", 1, 1).append(new MechanismLigament2d(
            "Bottom Side",
            4,
            -90,
            10,
            new Color8Bit(Color.kYellow)));
        
        // top right node
        elevator2d.getRoot("Outer Top Right", 1, 15).append(new MechanismLigament2d(
            "Top Side",
            4,
            -90,
            10,
            new Color8Bit(Color.kYellow)));

        elevator2d.getRoot("Ba", 5, 1).append(new MechanismLigament2d(
            "Right Side",
            14,
            0,
            10,
            new Color8Bit(Color.kYellow)));
        

        //innerFrame

        // bottom left node
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
        
        // top right node
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
        elevator2d.getRoot("Outer Bottom Left")
    }
}