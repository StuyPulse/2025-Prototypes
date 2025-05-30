package com.stuypulse.robot.subsystems.differentialWrist;


import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class DifferentialWristVisualizer {

    public static DifferentialWristVisualizer instance;

    static {
        instance = new DifferentialWristVisualizer();
    }

    public static DifferentialWristVisualizer getInstance() {
        return instance;
    }

    private final Mechanism2d canvas;

    private final MechanismRoot2d pitchRoot;
    private final MechanismLigament2d pitch;
    private final MechanismLigament2d pitchLeft;
    private final MechanismLigament2d pitchRight;
    // private final MechanismLigament2d base1;


    private final MechanismRoot2d rollRoot;
    private final MechanismLigament2d roll;
    private final MechanismLigament2d base2;

    public DifferentialWristVisualizer() {
        int width = 30;
        int height = 10;
        canvas = new Mechanism2d(width, height);

        pitchRoot = canvas.getRoot("Pitch Movement", width/3, height/3);
        pitch = new MechanismLigament2d("Pitch", 5, 0, 3, new Color8Bit(Color.kPurple));
        
        pitchLeft = new MechanismLigament2d("Pitch Left", 2.1, 0, 5, new Color8Bit(Color.kRed));
        pitchRight = new MechanismLigament2d("Pitch Right", 2, 0, 5, new Color8Bit(Color.kBlue));

        // base1 = new MechanismLigament2d("base", 2, 0, 5, new Color8Bit(Color.kRed));

        pitchRoot.append(pitch);
        pitchRoot.append(pitchLeft);
        pitchRoot.append(pitchRight);
        // pitchRoot.append(base1);

        rollRoot = canvas.getRoot("Roll Movement", 2*width/3, height/3);
        roll = new MechanismLigament2d("Roll", 5, 0, 3, new Color8Bit(Color.kOrange));
        
        base2 = new MechanismLigament2d("base", 5, 0, 3, new Color8Bit(Color.kOrange));
        
        rollRoot.append(roll);
        rollRoot.append(base2);
        
    }

    public void updateVisualizer(){
        DifferentialWrist wrist = DifferentialWrist.getInstance();

        SmartDashboard.putData("Visualizers/Differential Wrist", canvas);

        
        pitch.setAngle(wrist.getCurrentPitchAngle());
        roll.setAngle(wrist.getCurrentRollAngle()); 

        
        pitchRight.setAngle(wrist.getRightCurrentAngle());
        pitchLeft.setAngle(wrist.getLeftCurrentAngle());

    }
}
