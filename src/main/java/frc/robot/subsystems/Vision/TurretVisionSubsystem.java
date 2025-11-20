package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import javax.sql.rowset.BaseRowSet;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
// import frc.robot.util.E;
import frc.robot.util.PolynomialRegression;
// import frc.robot.util.EulerConversion;
public class TurretVisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private static AprilTagFieldLayout aprilTagFieldLayout;

    private NetworkTableInstance ntInstance;
    private NetworkTable visionStatsTable;
    private StructPublisher<Pose2d> visionPosePublisher;
    private DoublePublisher visionDistPublisher;
    private StructPublisher<Pose3d> cameraPosePublisher;
    private DoubleArrayPublisher tagDistancePublisher;
    private DoublePublisher desiredAnglePublisher;
    private DoublePublisher desiredTurrAnglePublisher;
    private DoublePublisher currentAnglePublisher;
    private StructPublisher<Rotation3d> cameraRotPublisher;

    private StructLogEntry<Pose2d> estimatedPoseLogEntry;
    private DoubleArrayLogEntry tagDistanceLogEntry;
    private IntegerArrayLogEntry tagIDLogEntry;

    private Consumer<TimestampedVisionUpdate> visionConsumer = (x) -> {};
    private final Servo baseServo;
    // private final Servo turrServo;
    private double angle;
    private double angleError;
    private double azimuth;
    private double angleSweepSpeed = 5;
    private double servoFOV = 150;
    private double azimuthSweepSpeed = 5;
    private int waitForCameraToMoveConst = 0;//thing that  prevents images to be captured while the turret is moving
    private double azimuthError;
    PhotonTrackedTarget closestTarget;
    private double lastTimestamp = 0.0;
    int hasFound = 0;
    boolean hasFoundAtleast1 = false;
    boolean autoMode = true;
    private PolynomialRegression xStdDevModel = VisionConstants.xStdDevModel;
    private PolynomialRegression yStdDevModel = VisionConstants.yStdDevModel;
    private PolynomialRegression oStdDevModel = VisionConstants.oStdDevModel;
    public void setCameraPose(double newAngle, double newTurrAngle){
        newAngle = Math.toRadians(baseServo.getAngle()*servoFOV*2/180 - servoFOV);
        // newTurrAngle = Math.toRadians(turrServo.getAngle()*servoFOV*2/180 - servoFOV);

        // Rotation3d newCamRotation = EulerConversion.zxyToXyz(newAngle,0.0,0.0);

        photonPoseEstimator.setRobotToCameraTransform(new Transform3d(0,0,0,new Rotation3d(0,0,-newAngle)));

        // angle  = newAngle;
        // azimuth = newTurrAngle;
        // cameraRotPublisher.set(newCamRotation);

    }
    //take in -135 deg to 135 deg angles
    public void setBaseAngle(double newAngle){
        newAngle = -newAngle;
        newAngle += servoFOV;
        newAngle *= 180.0/(servoFOV*2);
        baseServo.setAngle(newAngle);
    }
    public void setTurrAngle(double newAngle){
        newAngle = -newAngle;

        newAngle += servoFOV;
        newAngle *= 180.0/(servoFOV*2);

        // turrServo.setAngle(newAngle);
    }
    public void moveServo(){
        // System.out.println("moveServo");
        double currentTimestamp = Timer.getFPGATimestamp();
        double deltaT = currentTimestamp-lastTimestamp;
        lastTimestamp = currentTimestamp;
        
        Translation3d translation = 
        closestTarget.getBestCameraToTarget().getTranslation();

        angleError = Math.toDegrees(Math.atan(translation.getY()/translation.getX()));
        // azimuthError = Math.toDegrees(Math.atan(translation.getZ()/translation.getX()));
        System.out.print("Angle Error: ");
        System.out.println(angleError);
        // angleError = angleError;
        // azimuthError = azimuthError / 5;
        angle += angleError/2;
        // azimuth += azimuthError;
        // System.out.println(azimuth);
        // waitForCameraToMoveConst = (int)((angleError /(300.0/1.0))/0.02);
        //waitForCameraToMoveConst = 10;
        if (autoMode){
        setCameraPose(Math.toRadians(angle),Math.toRadians(azimuth));

        //azimuth += azimuthError/2;
        if (angle > servoFOV){angle = servoFOV;}else if (angle < -servoFOV){angle = -servoFOV;}
        if (azimuth > servoFOV){azimuth = servoFOV;}else if (azimuth < -servoFOV){azimuth = -servoFOV;}
        
        setBaseAngle(angle);
        // setTurrAngle(azimuth);
        }
    }
    public void sweep(){
        if (autoMode){
        angle = 0;
        // angle += angleSweepSpeed;
        // azimuth += azimuthSweepSpeed;
        if (angle >= servoFOV){angle = servoFOV;angleSweepSpeed = -5;}else if (angle <= -servoFOV){angle = -servoFOV;angleSweepSpeed = 5;}
        if (azimuth >= servoFOV){azimuth = servoFOV;azimuthSweepSpeed = -3;}else if (azimuth <= -servoFOV){azimuth = -servoFOV;azimuthSweepSpeed = 3;}
        setBaseAngle(angle);
        // setTurrAngle(azimuth);

        }
    }
    public TurretVisionSubsystem(CameraConfig cameraConfig) {
        // Initialize the camera with its name
        baseServo = new Servo(0);
        // turrServo = new Servo(1);
        // azimuth = 90;
        setBaseAngle(0);
        // setTurrAngle(0);
        lastTimestamp = Timer.getFPGATimestamp();
        camera = new PhotonCamera(cameraConfig.getCameraName());

        // Load AprilTag field layout 
        try{
            aprilTagFieldLayout = new AprilTagFieldLayout(
                Filesystem.getDeployDirectory() + "/2025-reefscape.json"
            );
        }
        catch (Exception e){
            throw new RuntimeException("Failed to load field layout", e);
        }

        // Create pose estimator
        photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            cameraConfig.getPoseStrategy(),
            cameraConfig.getCameraPose()
        );

        initNT(cameraConfig);
        initLog(cameraConfig);
    }

    @Override
    public void periodic() {
        // Get all unread results in the queue from the camera 
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        desiredAnglePublisher.set(angle);
        // desiredTurrAnglePublisher.set(azimuth);

        currentAnglePublisher.set(angle);
        //Loops through all unread results
        hasFoundAtleast1 = false;

        for (PhotonPipelineResult result : results){
            
            //checks if the camera detected any apriltags
            if (result.hasTargets()){
                hasFound = 0;
                hasFoundAtleast1 = true;
                double minDistance = Double.MAX_VALUE;
                long[] tagIDs = new long[result.getTargets().size()];
                double[] tagDistances = new double[result.getTargets().size()];
                //loops through all detected targets from the camera
                for(int i = 0; i < result.getTargets().size(); i++){
                    PhotonTrackedTarget target = result.getTargets().get(i);

                    Translation3d translation = 
                        target.getBestCameraToTarget().getTranslation();
                        
                    double distance = Math.sqrt(
                        Math.pow(translation.getX(),2) +
                        Math.pow(translation.getY(),2) +
                        Math.pow(translation.getZ(),2) 
                    );
                    if (distance < minDistance){
                        minDistance = distance;
                        closestTarget = target;
                    }
                    tagIDs[i] = target.getFiducialId();
                    tagDistances[i] = distance;
                }

            
                tagDistancePublisher.set(tagDistances);
                // tagDistanceLogEntry.append(tagDistances);
                // tagIDLogEntry.append(tagIDs);
                visionDistPublisher.set(minDistance);

                //Don't use vision measurement if tags are too far
                if(minDistance > 2) continue;

                Optional<EstimatedRobotPose> estimatedPose = 
                    photonPoseEstimator.update(result);
                if(estimatedPose.isPresent()){
                    Pose2d estimatedPose2d = 
                        estimatedPose.get().estimatedPose.toPose2d();
                    
                    //checks if the robot is in the field
                    double x = estimatedPose2d.getTranslation().getX();
                    double y = estimatedPose2d.getTranslation().getY();
                    if (x - VisionConstants.ROBOT_RADIUS < 0 ||
                        x + VisionConstants.ROBOT_RADIUS > VisionConstants.FIELD_X || 
                        y - VisionConstants.ROBOT_RADIUS < 0 ||
                        y + VisionConstants.ROBOT_RADIUS > VisionConstants.FIELD_Y
                    ){
                        continue;
                    }
                        
                    visionConsumer.accept(
                        new TimestampedVisionUpdate(
                            result.getTimestampSeconds(),
                            estimatedPose2d,
                            VecBuilder.fill(//standard deviation matrix
                                xStdDevModel.predict(minDistance),
                                yStdDevModel.predict(minDistance),
                                oStdDevModel.predict(minDistance))
                        )
                    );
                    visionPosePublisher.set(estimatedPose2d);
                    estimatedPoseLogEntry.update(estimatedPose2d);
                                    }
            }

            //if picture did not see any april tag
            else{
                hasFound ++;
            }
        }


        if (hasFoundAtleast1){
            moveServo();
        }
    
    if (hasFound > 10){
        hasFound = 11;//prevent overflow
        System.out.println("SWEEP");
        sweep();
    }
}

    /**
     * Sets up interfaces between swerve subsystem and vision subsystem
     * @param consumer consumer to receive vision updates
     */
    public void setInterface(Consumer<TimestampedVisionUpdate> consumer){
        visionConsumer = consumer;//thiing for vision to interface with the swerve subsystem
    }

    /**
     * Initializes Networktables.
     */
    private void initNT(CameraConfig cameraConfig){
        ntInstance = NetworkTableInstance.getDefault();
        visionStatsTable = ntInstance.getTable(
            "Vision Debug" + cameraConfig.getCameraName()
        );
        visionPosePublisher = visionStatsTable.getStructTopic(
            "estimated pose", Pose2d.struct
        ).publish();

        visionDistPublisher = visionStatsTable.getDoubleTopic(
            "dist"
        ).publish();
        cameraPosePublisher = visionStatsTable.getStructTopic(
            "camera pose", Pose3d.struct
        ).publish();
        tagDistancePublisher = visionStatsTable.getDoubleArrayTopic(
            "Tag Distances"
        ).publish();
        desiredAnglePublisher = visionStatsTable.getDoubleTopic(
            "Desired Angle"
        ).publish();
        desiredTurrAnglePublisher = visionStatsTable.getDoubleTopic(
            "Desired Turr Angle"
        ).publish();

        currentAnglePublisher = visionStatsTable.getDoubleTopic(
            "Curr Angle"
        ).publish();
        cameraPosePublisher.set(
            new Pose3d().transformBy(cameraConfig.getCameraPose())
        );
        cameraRotPublisher =  visionStatsTable.getStructTopic("cam rot", Rotation3d.struct).publish();
    }

    /**
     * Initializes data logging.
     * @param cameraConfig configuration for the camera
     */
    private void initLog(CameraConfig cameraConfig){
        tagIDLogEntry = new IntegerArrayLogEntry(
            DataLogManager.getLog(),
            cameraConfig.getCameraName() + " Tag IDs"
        );
        tagDistanceLogEntry = new DoubleArrayLogEntry(
            DataLogManager.getLog(),
            cameraConfig.getCameraName() + " Tag Distances"

        );
        estimatedPoseLogEntry = StructLogEntry.create(
            DataLogManager.getLog(),
            cameraConfig.getCameraName() + " Estimated Pose",
            Pose2d.struct
        );
    }
}