package com.stuypulse.robot.commands.arm;

public class MoveToLevel extends InstantCommand {
    public static Command untilDone(double height) {
        return untilDone(height, Lift.MAX_HEIGHT_ERROR);
    }

    public static Command untilDone(double height, double epsilon) {
        return new ArmtoAngle(height)
            .andThen(new WaitUntilCommand(() -> Amper.getInstance().isAtTargetHeight(epsilon)));
    }

    public static Command untilBelow(double height, double epsilon) {
        return new AmperToHeight(height)
            .andThen(new WaitUntilCommand(() -> Amper.getInstance().isAtBelowTargetHeight(epsilon)));
    }

    private final Arm arm;
    private final double angle;

    public AmperToHeight(double height) {
        amper = Amper.getInstance();
        this.height = height;

        addRequirements(amper);
    }

    @Override
    public void initialize() {
        amper.setTargetHeight(height);
    }
}