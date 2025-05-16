package com.stuypulse.robot.subsystems.hooded;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class TurretVisualizer {
    private MechanismObject2d root;
    private MechanismLigament2d turret;
    private Mechanism2d plane;

    public TurretVisualizer(){
        plane = new Mechanism2d(20, 20); //TODO update latr
        root = plane.getRoot("turret", 10, 10);
        turret = new MechanismLigament2d(
            "turretAngle",
            4, 
            0,
            1,
            new Color8Bit("#aaaa00"));

        root.append(turret);
    }

    public void updateAngle(Rotation2d angle){
        turret.setAngle(angle);
    }
}
