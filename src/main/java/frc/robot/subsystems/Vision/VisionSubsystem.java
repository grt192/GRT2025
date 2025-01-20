package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Vision.TimestampedVisionUpdate;
import frc.robot.util.PolynomialRegression;
public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private Pose2d currentPose = new Pose2d();
    private static AprilTagFieldLayout aprilTagFieldLayout;
    private static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
        // Modify these values based on your robot's camera mounting position
        0.5, 0.0, 0.5,  // x, y, z in meters
        new Rotation3d(0, 0, 0)  // roll, pitch, yaw in radians
    );

    private NetworkTableInstance ntInstance;
    private NetworkTable visionStatsTable;
    private StructPublisher visionPosePublisher;
    private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};
    
    private List<TimestampedVisionUpdate> cameraVisionUpdate;

    private PolynomialRegression xStdDevModel =
        new PolynomialRegression(VisionConstants.STD_DEV_DIST,VisionConstants.X_STD_DEV,2);
    private PolynomialRegression yStdDevModel =
        new PolynomialRegression(VisionConstants.STD_DEV_DIST,VisionConstants.Y_STD_DEV,2);
    private PolynomialRegression oStdDevModel = //standard deviation of the theta
        new PolynomialRegression(VisionConstants.STD_DEV_DIST,VisionConstants.O_STD_DEV,1);

    public VisionSubsystem() {
        // Initialize the camera with its network table name
        camera = new PhotonCamera("1");

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
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            CAMERA_TO_ROBOT
        );

        ntInstance = NetworkTableInstance.getDefault();
        visionStatsTable = ntInstance.getTable("VisionStats");
        visionPosePublisher = visionStatsTable.getStructTopic(
            "VisionPose", Pose2d.struct
        ).publish();
    }

    @Override
    public void periodic() {
        // Get the latest pipeline result the results are queued up since the last call
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        
        for (PhotonPipelineResult result : results){
            cameraVisionUpdate = new ArrayList<>();
            double timestamp = result.getTimestampSeconds();//get time stamp of results

            double totalDistance = 0.0;
            for (Pose3d tagPose : tagPose3ds) {
                totalDistance += tagPose.getTranslation().getDistance(cameraPose.getTranslation());
            }
            double avgDistance = totalDistance / tagPose3ds.size();
            double xStdDev = 
            if (result.hasTargets()) {
                // Get estimated robot pose
                Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(result);
                
                if (estimatedPose.isPresent()) {
                    // Update the current pose
                    currentPose = estimatedPose.get().estimatedPose.toPose2d();
                    // You might wanst to send this to other subsystems or log it
                }

            }
            cameraVisionUpdate.add(
            new TimestampedVisionUpdate(
                currentPose, //current x, y, and theta 
                timestamp,
                VecBuilder.fill(//standard deviation matrix
                    singleTagAdjustment * singleTagStdDevScalar * xyStdDev * stdDevScalarAuto,
                    singleTagAdjustment * singleTagStdDevScalar * xyStdDev * stdDevScalarAuto,
                    singleTagAdjustment * singleTagStdDevScalar * thetaStdDev * stdDevScalarAuto))
        )
        visionConsumer.accept(cameraVisionUpdate);

        }
        

    }

    /**
     * Gets the latest estimated robot pose from vision
     * @return The estimated Pose2d of the robot
     */
    public Pose2d getCurrentPose() {
        return currentPose;
    }

    /**
     * Checks if the camera currently sees any AprilTags
     * @return true if targets are detected
     */
    public boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    /**
     * Sets up interfaces between swerve subsystem and vision subsystem
     * @param consumer consumer to receive vision updates
     */
    public void setInterface(Consumer<List<TimestampedVisionUpdate>> consumer){
        visionConsumer = consumer;//thiing for vision to interface with the swerve subsystem
    }
}