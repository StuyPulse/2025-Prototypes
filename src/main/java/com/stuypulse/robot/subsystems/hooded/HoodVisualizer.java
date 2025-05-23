package com.stuypulse.robot.subsystems.hooded;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class HoodVisualizer {
    private MechanismObject2d root;
    private MechanismLigament2d hood;
    private Mechanism2d plane;

    public HoodVisualizer(){
        plane = new Mechanism2d(40, 40); //TODO update later
        root = plane.getRoot("hood", 20, 20);
        hood = new MechanismLigament2d(
            "hoodAngle",
            8, 
            0,
            2,
            new Color8Bit("#ffff00"));

        root.append(hood);

        SmartDashboard.putData("hood", plane);
    }

    public void updateAngle(Rotation2d angle){
        hood.setAngle(angle);
    }
}
