package com.stuypulse.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmVisualizer {

    public Arm arm;

    private final Mechanism2d stick;

    private final MechanismLigament2d stickStick;


    

    public ArmVisualizer() {
        arm = Arm.getInstance();
        stick = new Mechanism2d(10, 10);
        stickStick = new MechanismLigament2d(
        "arm",
        5,
        0,
        10,
        new Color8Bit(Color.kAqua));

        stick.getRoot("Root Origin", 0, 0).append(stickStick);
    }

    public void update() {
        stickStick.setAngle(arm.getArmAngle()); 
    }
}

