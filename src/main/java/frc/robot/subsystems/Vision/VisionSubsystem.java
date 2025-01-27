package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
    private Pose3d cameraPose = new Pose3d();
    private static AprilTagFieldLayout aprilTagFieldLayout;

    private NetworkTableInstance ntInstance;
    private NetworkTable visionStatsTable;
    private StructPublisher<Pose2d> visionPosePublisher;
    private DoublePublisher visionDistPublisher;
    private StructPublisher<Pose3d> cameraPosePublisher;

    private Consumer<TimestampedVisionUpdate> visionConsumer = (x) -> {};
    
    private PolynomialRegression xStdDevModel = VisionConstants.xStdDevModel;
    private PolynomialRegression yStdDevModel = VisionConstants.yStdDevModel;
    private PolynomialRegression oStdDevModel = VisionConstants.oStdDevModel;

    private static double robotRadius = 0.762;
    public VisionSubsystem(CameraConfig cameraConfig) {
        // Initialize the camera with its network table name
        camera = new PhotonCamera(cameraConfig.getCameraName());

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

        cameraPose = cameraPose.transformBy(cameraConfig.getCameraPose());

        initNT(cameraConfig.getCameraName());

    }

    @Override
    public void periodic() {
    //     // Get the latest pipeline result the results are queued up since the last call
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
                double x = estimatedPose.get().
                estimatedPose.toPose2d().getTranslation().getX();
                double y = estimatedPose.get().
                estimatedPose.toPose2d().getTranslation().getY();

                if(estimatedPose.isPresent()){

                    if ((x-robotRadius > 0) && (x+robotRadius < 17.548) && 
                        (y-robotRadius > 0) && (y+robotRadius < 8.052));
                        
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
                visionPosePublisher.set(estimatedPose.get().estimatedPose.toPose2d());
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
    private void initNT(String cameraName){
        ntInstance = NetworkTableInstance.getDefault();
        visionStatsTable = ntInstance.getTable("Vision Debug" + cameraName);
        visionPosePublisher = visionStatsTable.getStructTopic(
            "estimated pose", Pose2d.struct
        ).publish();
        visionDistPublisher = visionStatsTable.getDoubleTopic("dist").publish();
        cameraPosePublisher = visionStatsTable.getStructTopic(
            "camera pose", Pose3d.struct
        ).publish();
        cameraPosePublisher.set(cameraPose);
    }
}