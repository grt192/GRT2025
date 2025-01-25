package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.PolynomialRegression;
public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private Pose2d currentPose = new Pose2d();
    private static AprilTagFieldLayout aprilTagFieldLayout;
    private static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
        // Modify these values based on your robot's camera mounting position
        0.31, 0.0 ,0.,  // x, y, z in meters
        new Rotation3d(- Math.PI / 2., 0, 0)  // roll, pitch, yaw in radians
    );

    private NetworkTableInstance ntInstance;
    private NetworkTable visionStatsTable;
    private StructPublisher<Pose2d> visionPosePublisher;
    private DoublePublisher visionDistPublisher;

    private Consumer<TimestampedVisionUpdate> visionConsumer = (x) -> {};
    
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
            PoseStrategy.LOWEST_AMBIGUITY,
            CAMERA_TO_ROBOT
        );

        initNT();
    }

    @Override
    public void periodic() {
        // Get the latest pipeline result the results are queued up since the last call
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        
        //loops through all unread results
        for (PhotonPipelineResult result : results){
            
            //checks if the camera detected any apriltags
            if (result.hasTargets()){
                
                double minDistance = Double.MAX_VALUE; // Start with a very large value
        
                //loops through all detected targets from the camera
                for (PhotonTrackedTarget target : result.getTargets()) {
                    
                    Transform3d cameraToTagTransform = target.getBestCameraToTarget();
                    Translation3d translation = cameraToTagTransform.getTranslation();
                    double distance = Math.sqrt(
                        Math.pow(translation.getX(),2) +
                        Math.pow(translation.getY(),2) +
                        Math.pow(translation.getZ(),2) 
                    );
                    if (distance < minDistance){
                        minDistance = distance;
                    }
                }

                //Don't use vision measurement if tags are too far
                if(minDistance > 3) continue;

                Optional<EstimatedRobotPose> estimatedPose = 
                    photonPoseEstimator.update(result);
                if(estimatedPose.isPresent()){
                    visionConsumer.accept(
                        new TimestampedVisionUpdate(
                            result.getTimestampSeconds(),
                            estimatedPose.get().estimatedPose.toPose2d(),
                            VecBuilder.fill(//standard deviation matrix
                                xStdDevModel.predict(minDistance),
                                yStdDevModel.predict(minDistance),
                                oStdDevModel.predict(minDistance))
                        )
                    );
                }
               
                visionDistPublisher.set(minDistance);
                visionPosePublisher.set(currentPose);
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
    private void initNT(){
        ntInstance = NetworkTableInstance.getDefault();
        visionStatsTable = ntInstance.getTable("Vision Debug");
        visionPosePublisher = visionStatsTable.getStructTopic(
            "estimated pose", Pose2d.struct
        ).publish();
        visionDistPublisher = visionStatsTable.getDoubleTopic("dist").publish();
    }
}