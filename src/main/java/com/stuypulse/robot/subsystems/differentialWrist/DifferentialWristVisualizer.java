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

    private final MechanismRoot2d rollRoot;
    private final MechanismLigament2d roll;

    public DifferentialWristVisualizer() {
        int width = 10;
        int height = 10;
        canvas = new Mechanism2d(width, height);

        pitchRoot = canvas.getRoot("Pitch Movement", width/3, height/3);
        pitch = new MechanismLigament2d("Pitch", 5, 0, 1, new Color8Bit(Color.kBlue));
        pitchRoot.append(pitch);

        rollRoot = canvas.getRoot("Roll Movement", 2*width/3, height/3);
        roll = new MechanismLigament2d("Roll", 5, 0, 1, new Color8Bit(Color.kYellow));
        rollRoot.append(roll);
    }


    public void visualizerPeriodic() {
        DifferentialWrist diffWrist = DifferentialWrist.getInstance();

        SmartDashboard.putData("Visualizers/Differential Wrist", canvas);

        pitch.setAngle(diffWrist.getCurrentPitchAngle());
        roll.setAngle(diffWrist.getCurrentRollAngle()); 
        
    }
}
