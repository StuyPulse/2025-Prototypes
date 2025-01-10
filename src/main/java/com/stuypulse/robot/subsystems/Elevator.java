package com.stuypulse.robot.subsystems;

import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

class Elevator extends SubsystemBase {

     private final SmartNumber targetHeight;
    private double minHeight;

    private final Mechanism2d mechanism2d;
    private final MechanismLigament2d lift2d;

    public Elevator() {
        targetHeight = new SmartNumber("Elevator/Target Height", 0);
        

        
    }


}