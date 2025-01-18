package frc.robot.subsystems.pivot;

import frc.robot.Constants;

public enum PivotState {
    ZERO(Constants.IntakeConstants.ZERO_POSITION),
    OUTTAKE(Constants.IntakeConstants.OUTTAKE_POSITION),
    SOURCE(Constants.IntakeConstants.SOURCE_POSITION),
    VERTICAL(Constants.IntakeConstants.VERTICAL_POSITION);

    private final double targetAngle;

    private PivotState(double targetAngle) {
        this.targetAngle = targetAngle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

}
