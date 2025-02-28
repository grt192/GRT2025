package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.PolynomialRegression;
public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private static AprilTagFieldLayout aprilTagFieldLayout;

    private NetworkTableInstance ntInstance;
    private NetworkTable visionStatsTable;
    private StructPublisher<Pose2d> visionPosePublisher;
    private DoublePublisher visionDistPublisher;
    private StructPublisher<Pose3d> cameraPosePublisher;
    private DoubleArrayPublisher tagDistancePublisher;
    private StructLogEntry<Pose2d> estimatedPoseLogEntry;
    private DoubleArrayLogEntry tagDistanceLogEntry;
    private IntegerArrayLogEntry tagIDLogEntry;

    private Consumer<TimestampedVisionUpdate> visionConsumer = (x) -> {};
    
    private PolynomialRegression xStdDevModel = VisionConstants.xStdDevModel;
    private PolynomialRegression yStdDevModel = VisionConstants.yStdDevModel;
    private PolynomialRegression oStdDevModel = VisionConstants.oStdDevModel;

    public VisionSubsystem(CameraConfig cameraConfig) {
        // Initialize the camera with its name
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
        
        //Loops through all unread results
        for (PhotonPipelineResult result : results){
            
            //checks if the camera detected any apriltags
            if (result.hasTargets()){
                
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
                    }
                    tagIDs[i] = target.getFiducialId();
                    tagDistances[i] = distance;
                }
                tagDistancePublisher.set(tagDistances);
                tagDistanceLogEntry.append(tagDistances);
                tagIDLogEntry.append(tagIDs);
                visionDistPublisher.set(minDistance);

                //Don't use vision measurement if tags are too far
                if(minDistance > 2) continue;

                Optional<EstimatedRobotPose> estimatedPose = 
                    photonPoseEstimator.update(result);
                if(estimatedPose.isPresent()){
                    Pose2d estimatedPose2d = 
                        estimatedPose.get().estimatedPose.toPose2d();

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
        cameraPosePublisher.set(
            new Pose3d().transformBy(cameraConfig.getCameraPose())
        );
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