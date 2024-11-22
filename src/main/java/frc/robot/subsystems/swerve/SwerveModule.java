package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    private final KrakenDriveMotor driveMotor;
    private final NeoSteerMotor steerMotor;

    private double offsetRads;

    private static final double DRIVE_P = 1; // temporary value
    private static final double DRIVE_I = 0;
    private static final double DRIVE_D = 0;
    private static final double DRIVE_FF = 0;

    private static final double STEER_P = 1; // temporary value
    private static final double STEER_I = 0;
    private static final double STEER_D = 0;
    private static final double STEER_FF = 0;


    /** Constructs a Swerve Module.
     *
     * @param drivePort The CAN ID of the drive motor
     * @param steerPort The CAN ID of the steer motor
     * @param offsetRads The offset of the absolute encoder
     */

    public SwerveModule(int drivePort, int steerPort, double offsetRads) {

        steerMotor = new NeoSteerMotor(steerPort);
        steerMotor.configurePID(STEER_P, STEER_I, STEER_D, STEER_FF);

        driveMotor = new KrakenDriveMotor(drivePort);
        driveMotor.configPID(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_FF);

        this.offsetRads = offsetRads;

    }

    /** Sets the desired state of this swerve module through setting the PID targets.
     *
     * @param state The desired SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState state) {
        // Rotation2d currentAngle = getWrappedAngle();
        // SwerveModuleState optimized = SwerveModuleState.optimize(state, currentAngle);

        // double targetAngleRads = optimized.angle.getRadians() - offsetRads;
        // double angleErrorRads = optimized.angle.minus(currentAngle).getRadians();

        // // Multiply by cos so we don't move quickly when the swerves are angled wrong
        // double targetVelocity = optimized.speedMetersPerSecond * Math.cos(angleErrorRads);

        driveMotor.setVelocity(state.speedMetersPerSecond);
        steerMotor.setPosition(state.angle.getRadians()); //OFSET RADS
    }

    /** Sets the raw powers of the swerve module.
     *
     * @param drivePower The power for the drive motor
     * @param steerPower The power for the steer motor
     */
    public void setRawPowers(double drivePower, double steerPower) {
        driveMotor.setPower(drivePower);
        steerMotor.setPower(steerPower);
    }

    /** Gets the current state of the swerve module.
     *
     * @return The current SwerveModulePosition of this module
     */
    public SwerveModulePosition getState() {
        return new SwerveModulePosition(
            driveMotor.getDistance(),
            getWrappedAngle()
        );
    }

    /** Gets the current angle of the module.
     *
     * @return Wrapped angle in radians from -pi to pi
     */
    public Rotation2d getWrappedAngle() {
        double angleRads = steerMotor.getPosition();
        double wrappedAngleRads = MathUtil.angleModulus(angleRads + offsetRads);

        return new Rotation2d(wrappedAngleRads);
    }

    
    /** Gets the error of the drive motor.
     *
     * @return The velocity PID error of the drive motor.
     */
    public double getDriveError() {
        return driveMotor.getError();
    }

    /** Gets the setpoint of the drive motor.
     *
     * @return The current setpoint of the drive motor.
     */
    public double getDriveSetpoint() {
        return driveMotor.getSetpoint();
    }

    /** Gets the current amp draw of the drive motor.
     *
     * @return The amp draw of the drive motor.
     */
    public double getDriveAmpDraw() {
        return driveMotor.getAmpDraw();
    }

    /** Gets the distance the distance driven by the drive motor.
     *
     * @return The distance driven in meters.
     */
    public double getDistanceDriven() {
        return driveMotor.getDistance();
    }

    /** Gets the velocity of the drive motor.
     *
     * @return The velocity of the drive motor in meters/second.
     */
    public double getDriveVelocity() {
        return driveMotor.getVelocity();
    }
}
