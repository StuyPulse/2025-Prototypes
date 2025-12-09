package com.stuypulse.robot.constants;

public class Gains {
    public interface HDSR {
        double kP = 1.0;
        double kI = 0.0;
        double kD = 0.1;
    }

    public interface pidToPose {
        public interface x {
            double kP = 1.5;
            double kI = 0.0;
            double kD = 0.05;
        }

        public interface y {
            double kP = 1.0;
            double kI = 0.0;
            double kD = 0.0;
        }

        public interface theta {
            double kP = 1.0;
            double kI = 0.0;
            double kD = 0.0;
        }
        
    }
}
