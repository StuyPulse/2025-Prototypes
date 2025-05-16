package com.stuypulse.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;


public interface Constants {
    public interface Telescope{
        double MIN_HEIGHT_METERS = Units.inchesToMeters(0);
        double MAX_HEGIHT_METERS = Units.inchesToMeters(0); 
        
        //Units TBD
        double ARM_LENGTH = 0;
        double LIG_ONE_MAX = 0;
        double LIG_TWO_MAX = 0;
    }

}
