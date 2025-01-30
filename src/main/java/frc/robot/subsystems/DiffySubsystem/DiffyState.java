package frc.robot.subsystems.DiffySubsystem;

import frc.robot.Constants.DiffyConstants;

/** States of the elevator. */
public enum DiffyState {
    ANGLE_ZERO(DiffyConstants.ARM_ANGLE_0, DiffyConstants.WRIST_ANGLE_0 ),
    ANGLE_CORAL(DiffyConstants.ARM_ANGLE_CORAL, DiffyConstants.WRIST_ANGLE_CORAL),
    ANGLE_90(DiffyConstants.ARM_ANGLE_90, DiffyConstants.WRIST_ANGLE_90),
    ANGLE_180(DiffyConstants.ARM_ANGLE_180, DiffyConstants.WRIST_ANGLE_180 );

    private final double armAngle;
    private final double wristAngle;

    private DiffyState(double armPos, double wristPos) {
        this.armAngle = armPos;
        this.wristAngle = wristPos;
    }


    /**
     * Gets the angle of a state in ticks.
     *
     * @return meters in double.
     */
    public double getDiffyArmPos() {
        return this.armAngle;
    }

    public double getDiffyWristPos() {
        return this.wristAngle;
    }
}