package frc.robot.subsystems.Elevator;

import frc.robot.Constants;

/** States of the elevator. */
public enum ElevatorState {
    ZERO_STATE(Constants.ElevatorConstants.ZERO_STATE),
    SOURCE(Constants.ElevatorConstants.SOURCE_POSITION),
    ONE_DROP(Constants.ElevatorConstants.ONE_DROP),
    TWO_DROP(Constants.ElevatorConstants.TWO_DROP),
    THREE_DROP(Constants.ElevatorConstants.THREE_DROP),
    FOUR_DROP(Constants.ElevatorConstants.FOUR_DROP);

    private final double extendedDistanceMeters;

    private ElevatorState(double extendedDistanceMeters) {
        this.extendedDistanceMeters = extendedDistanceMeters;
    }

    /**
     * Gets the distance of a state in meters.
     *
     * @return meters in double.
     */
    public double getExtendedDistanceMeters() {
        return this.extendedDistanceMeters;
    }

}