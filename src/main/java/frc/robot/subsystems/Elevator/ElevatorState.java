package frc.robot.subsystems.Elevator;

import frc.robot.Constants.ElevatorConstants;

/** States of the elevator. */
public enum ElevatorState {
    ZERO_STATE(ElevatorConstants.ZERO_STATE),
    SOURCE(ElevatorConstants.SOURCE_POSITION),
    ONE_DROP(ElevatorConstants.ONE_DROP),
    TWO_DROP(ElevatorConstants.TWO_DROP),
    THREE_DROP(ElevatorConstants.THREE_DROP),
    FOUR_DROP(ElevatorConstants.FOUR_DROP);

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

    public double getExtendInTicks() { 
        return this.extendedDistanceMeters * ElevatorConstants.METERS_TO_TICKS;
    }

}