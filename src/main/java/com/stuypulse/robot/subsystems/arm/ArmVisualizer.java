package com.stuypulse.robot.subsystems.arm;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class ArmVisualizer{

    public static ArmVisualizer instance = new ArmVisualizer();
    
    private final Mechanism2d stick;
    private final MechanismRoot2d link;
    private final MechanismLigament2d stickStick;

    public static ArmVisualizer getInstance() {
        return instance;
    }
    public ArmVisualizer() {

        stick = new Mechanism2d(100, 100);

        stickStick = new MechanismLigament2d(
        "arm",
        20,
        90,
        10,
        new Color8Bit(Color.kAqua));

        link = stick.getRoot("Root Origin", 0, 0);
        link.append(stickStick);

    }

    public void update() {
        SmartDashboard.putData("Arm", stick);
        stickStick.setAngle(Arm.getInstance().getTargetAngle()); 
    }
}

