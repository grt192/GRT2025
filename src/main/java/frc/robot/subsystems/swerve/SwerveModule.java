package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

import static frc.robot.Constants.SwerveConstants.*;

import java.util.EnumSet;

public class SwerveModule {

    private final KrakenDriveMotor driveMotor;
    private final NeoSteerMotor steerMotor;

    private int drivePort;
    private int steerPort;

    private int driveIndex;
    private int steerIndex;

    private double offsetRads = 0;

    /** Constructs a Swerve Module.
     *
     * @param drivePort The CAN ID of the drive motor
     * @param steerPort The CAN ID of the steer motor
     * @param offsetRads The offset of the absolute encoder
     */

    public SwerveModule(int drivePort, int steerPort, double offsetRads) {

        this.drivePort = drivePort;
        this.steerPort = steerPort;

        driveIndex = drivePort / 2;
        steerIndex = (steerPort -1) / 2;

        steerMotor = new NeoSteerMotor(steerPort);
        steerMotor.configurePID(
            STEER_P[steerIndex],
            STEER_I[steerIndex],
            STEER_D[steerIndex],
            STEER_FF[steerIndex]
        );

        driveMotor = new KrakenDriveMotor(drivePort);
        driveMotor.configPID(
            DRIVE_P[driveIndex],
            DRIVE_I[driveIndex],
            DRIVE_D[driveIndex],
            DRIVE_S[driveIndex],
            DRIVE_V[driveIndex]
        );

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

    public void publishDriveStats(){
        driveMotor.publishStats();
    }

    public void publishSteerStats(){
        steerMotor.publishStats();
    }

    public void steerDebug(){
        NetworkTableInstance.getDefault().getTable("steerDebug")
            .getEntry(steerPort + "PIDF")
            .setDoubleArray(
                new double[] {
                    STEER_P[steerIndex],
                    STEER_I[steerIndex],
                    STEER_D[steerIndex],
                    STEER_FF[steerIndex]
                }
            );
        NetworkTableInstance.getDefault().getTable("steerDebug").addListener(
            steerPort + "PIDF",
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (table, key, event) -> {
                double[] pidf = event.valueData.value.getDoubleArray();
                steerMotor.configurePID(pidf[0], pidf[1], pidf[2], pidf[3]);
            }
        );
    }

    public void driveDebug(){
        NetworkTableInstance.getDefault().getTable("driveDebug")
            .getEntry(drivePort + "PIDSV")
            .setDoubleArray(
                new double[] {
                    DRIVE_P[driveIndex],
                    DRIVE_I[driveIndex],
                    DRIVE_D[driveIndex],
                    DRIVE_S[driveIndex],
                    DRIVE_V[driveIndex]
                }
            );
        NetworkTableInstance.getDefault().getTable("driveDebug").addListener(
            drivePort + "PIDSV",
            EnumSet.of(NetworkTableEvent.Kind.kValueAll),
            (table, key, event) -> {
                double[] pidsv = event.valueData.value.getDoubleArray();
                driveMotor.configPID(pidsv[0], pidsv[1], pidsv[2], pidsv[3], pidsv[4]);
            }
        );
    }
}
