package frc.robot.subsystems.Elevator;

import frc.robot.Constants.ElevatorConstants;

/** States of the elevator. */
public enum ElevatorState {
    GROUND(ElevatorConstants.GROUND),
    SOURCE(ElevatorConstants.SOURCE),
    L1(ElevatorConstants.L1), //currently at test vlue
    L2(ElevatorConstants.L2),
    L3(ElevatorConstants.L3),
    L4(ElevatorConstants.L4);

    private final double distance;

    private ElevatorState(double distance) {
        this.distance = distance;
    }

    /**
     * Gets the distance of a state in meters.
     *
     * @return meters in double.
     */
    public double getDistance() {
        return this.distance;
    }

    public double getDistanceInTicks() { 
        return this.distance * ElevatorConstants.dutyCycletoticks;
    }
}