package frc.robot.subsystems.swerve;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SwerveSetpoint;

import static frc.robot.Constants.SwerveConstants.BL_DRIVE;
import static frc.robot.Constants.SwerveConstants.BL_OFFSET;
import static frc.robot.Constants.SwerveConstants.BL_POS;
import static frc.robot.Constants.SwerveConstants.BL_STEER;
import static frc.robot.Constants.SwerveConstants.BR_DRIVE;
import static frc.robot.Constants.SwerveConstants.BR_OFFSET;
import static frc.robot.Constants.SwerveConstants.BR_POS;
import static frc.robot.Constants.SwerveConstants.BR_STEER;
import static frc.robot.Constants.SwerveConstants.FL_DRIVE;
import static frc.robot.Constants.SwerveConstants.FL_OFFSET;
import static frc.robot.Constants.SwerveConstants.FL_POS;
import static frc.robot.Constants.SwerveConstants.FL_STEER;
import static frc.robot.Constants.SwerveConstants.FR_DRIVE;
import static frc.robot.Constants.SwerveConstants.FR_OFFSET;
import static frc.robot.Constants.SwerveConstants.FR_POS;
import static frc.robot.Constants.SwerveConstants.FR_STEER;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

//krakens drive, neo steer

public class SwerveSubsystem extends SubsystemBase {

    private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    private SwerveModuleState[] states = {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private Rotation2d driverHeadingOffset = new Rotation2d();

    public static final double MAX_VEL = 6000 / 6.923 / 60 * 2 * 2 * Math.PI * .0254; // kraken speed / gear ratio / reduced to per second * circumference * convert to meter
    public static final double MAX_OMEGA = MAX_VEL / FL_POS.getNorm();

    private final AHRS ahrs;

    private NetworkTableInstance ntInstance;
    private NetworkTable swerveTable;
    private NetworkTableEntry swerveDesiredStatesEntry;
    private NetworkTableEntry swerveVxEntry;
    private NetworkTableEntry swerveVyEntry;
    private NetworkTableEntry swerveVoEntry;
    private NetworkTableEntry chassisVxEntry;
    private NetworkTableEntry chassisVyEntry;
    private NetworkTableEntry chassisVoEntry;
    private NetworkTableEntry swerveTestToggleEntry;
    private NetworkTableEntry swerveTestAngleEntry;

    private final Field2d fieldVisual = new Field2d();
    private final ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    // private final SwerveSetpointGenerator setpointGenerator;
    
    public SwerveSubsystem() {

        ahrs = new AHRS(NavXComType.kMXP_SPI);

        frontLeftModule = new SwerveModule(FL_DRIVE, FL_STEER, FL_OFFSET);
        frontRightModule = new SwerveModule(FR_DRIVE, FR_STEER, FR_OFFSET);
        backLeftModule = new SwerveModule(BL_DRIVE, BL_STEER, BL_OFFSET);
        backRightModule = new SwerveModule(BR_DRIVE, BR_STEER, BR_OFFSET);

        kinematics = new SwerveDriveKinematics(FL_POS, FR_POS, BL_POS, BR_POS);
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics, 
            getGyroHeading(), 
            getModulePositions(),
            new Pose2d()
            );
        
        ntInstance = NetworkTableInstance.getDefault();
        swerveTable = ntInstance.getTable("Swerve");
        swerveDesiredStatesEntry = swerveTable.getEntry("DesiredStates");
        swerveVxEntry = swerveTable.getEntry("swerveVx");
        swerveVyEntry = swerveTable.getEntry("swerveVy");
        swerveVoEntry = swerveTable.getEntry("swerveVo");
        chassisVxEntry = swerveTable.getEntry("chassisVx");
        chassisVyEntry = swerveTable.getEntry("chassisVy");
        chassisVoEntry = swerveTable.getEntry("chassisVo");
        swerveTestToggleEntry = swerveTable.getEntry("TestToggle");
        swerveTestToggleEntry.setBoolean(false);
        swerveTestAngleEntry = swerveTable.getEntry("TestAngle");
        
        tab.add("Field", fieldVisual);
        
    }

    @Override
    public void periodic() {
        frontLeftModule.setDesiredState(states[0]);
        frontRightModule.setDesiredState(states[1]);
        backLeftModule.setDesiredState(states[2]);
        backRightModule.setDesiredState(states[3]);
                
        Rotation2d gyroAngle = getGyroHeading();
        Pose2d estimate = poseEstimator.update(
            gyroAngle,
            getModulePositions()
        );
        
        fieldVisual.setRobotPose(getRobotPosition());
        // updateNT(); //Causing Loop Overrun
        //Causing Overrun
        // String[] strStates = new String[4];
        // for (int i = 0; i < 4; i++) {
        //     strStates[i] = states[i].toString();
        // }
        // swerveDesiredStatesEntry.setStringArray(strStates);
}

    /**
     * Sets the powers of the drivetrain through PIDs. Relative to the driver heading on the field.
     *
     * @param xPower [-1, 1] The forward power.
     * @param yPower [-1, 1] The left power.
     * @param angularPower [-1, 1] The rotational power.
     */
    public void setDrivePowers(double xPower, double yPower, double angularPower) {
        swerveVxEntry.setDouble(xPower);
        swerveVyEntry.setDouble(yPower);
        swerveVoEntry.setDouble(angularPower);
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xPower * MAX_VEL, 
            yPower * MAX_VEL, 
            angularPower * MAX_OMEGA,
            getDriverHeading()
        );
        chassisVxEntry.setDouble(speeds.vxMetersPerSecond);
        chassisVyEntry.setDouble(speeds.vyMetersPerSecond);
        chassisVoEntry.setDouble(speeds.omegaRadiansPerSecond);
        // System.out.println(speeds);

        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA);
        // System.out.println(states[0]);
    }

    /**
     * Gets the module positions.
     *
     * @return The array of module positions.
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        };
    }

    /**
     * Gets the driver heading.
     *
     * @return The angle of the robot relative to the driver heading.
     */
    public Rotation2d getDriverHeading() {

        Rotation2d robotHeading = ahrs.isConnected()
            ? getGyroHeading()
            : getRobotPosition().getRotation();
        
        return robotHeading.minus(driverHeadingOffset);
    }

    /**
     * Resets the driver heading.
     *
     * @param currentRotation The new driver heading.
     */
    public void resetDriverHeading(Rotation2d currentRotation) {
        driverHeadingOffset = getGyroHeading().minus(currentRotation);
    }

    /** Resets the driver heading to 0. */
    public void resetDriverHeading() {
        resetDriverHeading(new Rotation2d());
    }


    /** Gets the gyro heading.*/
    private Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(ahrs.getAngle()); //might be flipped, tune
    }

        /**
     * Gets the current robot pose.
     *
     * @return The robot Pose2d.
     */
    public Pose2d getRobotPosition() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Set all swerve modules to a test angle.
     * @param angle angle in radiants 
     */
    public void setTestAngle(double angle){
        // Setting through swerve state
        // SwerveModuleState testState =
        //     new SwerveModuleState(
        //         0., new Rotation2d(Math.toRadians(angle))
        //     );
        // frontLeftModule.setDesiredState(testState);
        // frontRightModule.setDesiredState(testState);
        // backLeftModule.setDesiredState(testState);
        // backRightModule.setDesiredState(testState);

        frontLeftModule.steerMotor.setPosition(angle);
        // frontRightModule.steerMotor.setPosition(angle);
        // backLeftModule.steerMotor.setPosition(angle);
        // backRightModule.steerMotor.setPosition(angle);
    }
    
    private void updateNT(){
        frontLeftModule.updateNT();
        frontRightModule.updateNT();
        backLeftModule.updateNT();
        backRightModule.updateNT();
    }
}
