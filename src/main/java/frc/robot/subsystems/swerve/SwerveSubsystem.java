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
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SwerveSetpoint;

import static frc.robot.Constants.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

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

    public static final double MAX_VEL = 6000 / 6.923 / 60 * 2 * 2 * Math.PI * .0254; // Kraken speed / gear ratio / reduced to per second * circumference * convert to meters
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

    private StructPublisher<Pose2d> estimatedPosePublisher;

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
        

        RobotConfig config = new RobotConfig(null, null, null, null);
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            // Handle exception as needed, maybe use default values or fallback
        }

        AutoBuilder.configure(
            this::getRobotPosition,
            this::resetPose,
            this::getRobotRelativeChassisSpeeds,
            (speeds, feedforwards) -> setRobotRelativeDrivePowers(speeds),
            
            new PPHolonomicDriveController(
                new PIDConstants(1.0, 0.0, 0.0),
                new PIDConstants(1.0, 0.0, 0.0)
            ),

            config,
            ()->{
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get()==DriverStation.Alliance.Red;
                }
                return false; 
            },
            this

        );

        estimatedPosePublisher = swerveTable.getStructTopic(
            "estimatedPose",
            Pose2d.struct
        ).publish();
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
        estimatedPosePublisher.set(estimate);
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
        Rotation2d robotHeading = ahrs.isConnected() ? getGyroHeading() : getRobotPosition().getRotation();
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
        return Rotation2d.fromDegrees(ahrs.getAngle()); // Might need to flip depending on the robot setup
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
    public void resetPose(Pose2d currentPose) {
        Rotation2d gyroAngle = getGyroHeading();
        poseEstimator.resetPosition(
            gyroAngle, 
            getModulePositions(), 
            currentPose
        );
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            kinematics.toChassisSpeeds(states),
            getRobotPosition().getRotation() // Could be replaced with getGyroHeading() if desired
        );
        return robotRelativeSpeeds;
    }
    
    public void setRobotRelativeDrivePowers(ChassisSpeeds robotRelativeSpeeds) {
        
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds,
            new Rotation2d(0)
        );

        states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
            states, speeds,
            MAX_VEL, MAX_VEL, MAX_OMEGA);
    }


}
