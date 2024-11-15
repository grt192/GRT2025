package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class SwerveModule {

    private final KrakenDriveMotor driveMotor;
    private final CANSparkMax steerMotor;

    private double offsetRads;

    private static final double DRIVE_ROTATIONS_PER_METER = 0;

    private static final double DRIVE_P = 1; // temporary value
    private static final double DRIVE_I = 0;
    private static final double DRIVE_D = 0;
    private static final double DRIVE_FF = 0;


    /** Constructs a Swerve Module.
     *
     * @param drivePort The CAN ID of the drive motor
     * @param steerPort The CAN ID of the steer motor
     * @param offsetRads The offset of the absolute encoder
     */

    public SwerveModule(int drivePort, int steerPort, double offsetRads) {

        steerMotor = new CANSparkMax(steerPort, MotorType.kBrushless);

        driveMotor = new KrakenDriveMotor(drivePort);
        driveMotor.configPID(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_FF);

        driveMotor.setPositionConversionFactor(DRIVE_ROTATIONS_PER_METER);
        driveMotor.setVelocityConversionFactor(DRIVE_ROTATIONS_PER_METER / 60.0);

        this.offsetRads = offsetRads;

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
        double angleRads = 0;//get steer angle here
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
