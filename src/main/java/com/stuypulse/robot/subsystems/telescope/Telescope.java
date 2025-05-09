import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Telescope extends SubsystemBase {

    public static final Telescope instance;

    static {
        instance = new Telescope();
    }

    public static Telescope getinstance() {
        return instance;
    }

    public enum targetHeights{
        L1(Settings.Telescope.L1),
        L2(Settings.Telescope.L2),

    public abstract void setTargetHeight(double height);
    public abstract double getTargetHeight();

    
    

}