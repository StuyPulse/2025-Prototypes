package com.stuypulse.robot.constants;

public interface Gains {
    public interface superStructure {
        public interface arm{
            public interface PID{
                double kP = 0;
                double kI = 0;
                double kD = 0;
            }
            public interface FF{
                double kS = 0;
                double kV = 0;
                double kA = 0;
                double kG = 0;
            }
        }
        public interface telescope{
            public interface PID{
                double kP = 0;
                double kI = 0;
                double kD = 0;
            }
            public interface FF{
                double kS = 0;
                double kV = 0;
                double kA = 0;
                double kG = 0;
            }
        }
    }
}