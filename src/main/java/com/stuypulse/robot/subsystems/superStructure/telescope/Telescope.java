import com.stuypulse.robot.Robot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public abstract class Telescope extends SubsystemBase {

    public static final Telescope instance;

    static {
        instance = new TelescopeImpl();
    }

    public static Telescope getinstance() {
        return instance;
    }

    public enum TelescopeState{
        L1(Settings.Telescope.L1_HEIGHT_METERS),
        L2_FRONT(Settings.Telescope.L2_FRONT_HEIGHT_METERS),
        L3_FRONT(Settings.Telescope.L3_FRONT_HEIGHT_METERS),
        L4_FRONT(Settings.Telescope.L4_FRONT_HEIGHT_METERS),
        L2_BACK(Settings.Telescope.L2_BACK_HEIGHT_METERS),
        L3_BACK(Settings.Telescope.L3_BACK_HEIGHT_METERS),
        L4_BACK(Settings.Telescope.L4_BACK_HEIGHT_METERS);
    
        private final Number targetHeight;

        private TelescopeState(Number targetHeight) {
            this.targetHeight = targetHeight;
    
    
        }
    
        public Number getTargetHeight(){
            return targetHeight;
        }

    }


 
    public abstract boolean atTargetHeight();

    
    @Override
    public void periodic() {
    }

}