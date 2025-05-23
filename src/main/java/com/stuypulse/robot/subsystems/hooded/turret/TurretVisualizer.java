package com.stuypulse.robot.subsystems.hooded.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class TurretVisualizer {
    private MechanismObject2d root;
    private MechanismLigament2d turret;
    private Mechanism2d plane;

    public TurretVisualizer(){
        plane = new Mechanism2d(40, 40); //TODO update later
        root = plane.getRoot("turret", 20, 20);
        turret = new MechanismLigament2d(
            "turretAngle",
            8, 
            0,
            2,
            new Color8Bit("#ffff00"));

        root.append(turret);

        SmartDashboard.putData("turret", plane);
    }

    public void updateAngle(Rotation2d angle){
        turret.setAngle(angle);
    }
}
