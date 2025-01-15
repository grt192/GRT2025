package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import static frc.robot.Constants.SwerveConstants.*;

public class SwerveModule {

    public final KrakenDriveMotor driveMotor;
    public final NeoSteerMotor steerMotor;

    private double offsetRads = 0;

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
        driveMotor.configPID(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_S, DRIVE_V);

        this.offsetRads = offsetRads;
    }

    /** Sets the un optimized desired state of this swerve module through setting the PID targets.
     *
     * @param state The desired SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState state) {
        Rotation2d currentAngle = getWrappedAngle();
        state.optimize(currentAngle);

        double targetAngleRads = state.angle.getRadians() - offsetRads;
        double angleErrorRads = state.angle.minus(currentAngle).getRadians();

        // Multiply by cos so we don't move quickly when the swerves are angled wrong
        double targetVelocity = state.speedMetersPerSecond * Math.cos(angleErrorRads);

        driveMotor.setVelocity(targetVelocity);
        steerMotor.setPosition(targetAngleRads);
    }

    /** Sets the optimized desired state of this swerve module through setting the PID targets.
     *
     * @param state The desired SwerveModuleState
     */
    public void setOptomizedDesiredState(SwerveModuleState state) {
        driveMotor.setVelocity(state.speedMetersPerSecond);
        steerMotor.setPosition(state.angle.getRadians()); 
    }

    /** Gets the current state of the swerve module.
     *
     * @return The current SwerveModulePosition of this module
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            driveMotor.getDistance(),
            getWrappedAngle()
        );
    }
    /**
     * Gets the state of the swerve module (drive velo in m/s + angle 0-1 )
     * @return state of the module (velo is m/s and angle is double from 0 to 1)
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(
            driveMotor.getVelocity(),
            getWrappedAngle()
        );
    }

    /** Gets the current angle of the module.
     *
     * @return Wrapped angle in radians from -pi to pi
     */
    public Rotation2d getWrappedAngle() {
        // returned a 0-1 value
        double angleDouble = steerMotor.getPosition();
        double angleRads = (2. * Math.PI * angleDouble) - Math.PI;
        // double wrappedAngleRads = MathUtil.angleModulus(angleRads + offsetRads);

        return new Rotation2d(angleRads);
    }

    /** Gets the error of the drive motor.
     *
     * @return The velocity PID error of the drive motor.
     */
    public double getDriveError() {
        return driveMotor.getError();
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
