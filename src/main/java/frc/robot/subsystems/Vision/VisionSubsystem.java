package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonPoseEstimator;
    private Pose2d currentPose = new Pose2d();
    private final Field2d fieldVisualization = new Field2d();
    private final ShuffleboardTab tab = Shuffleboard.getTab("Vision");
    private static final AprilTagFieldLayout aprilTagFieldLayout = 
      new AprilTagFieldLayout(
        List.of(
            new AprilTag(
                1,
                new Pose3d(
                16.697198000,
                0.655320000,
                1.485900000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                2,
                new Pose3d(
                16.697198000,
                7.396480000,
                1.485900000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                3,
                new Pose3d(
                11.560810000,
                8.055610000,
                1.301750000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                4,
                new Pose3d(
                9.276080000,
                6.137656000,
                1.867916000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                5,
                new Pose3d(
                9.276080000,
                1.914906000,
                1.867916000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                6,
                new Pose3d(
                13.474446000,
                3.306318000,
                0.308102000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                7,
                new Pose3d(
                13.890498000,
                4.025900000,
                0.308102000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                8,
                new Pose3d(
                13.474446000,
                4.745482000,
                0.308102000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                9,
                new Pose3d(
                12.643358000,
                4.745482000,
                0.308102000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                10,
                new Pose3d(
                12.227306000,
                4.025900000,
                0.308102000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                11,
                new Pose3d(
                12.643358000,
                3.306318000,
                0.308102000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                12,
                new Pose3d(
                0.851154000,
                0.655320000,
                1.485900000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                13,
                new Pose3d(
                0.851154000,
                7.396480000,
                1.485900000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                14,
                new Pose3d(
                8.272272000,
                6.137656000,
                1.867916000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                15,
                new Pose3d(
                8.272272000,
                1.914906000,
                1.867916000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                16,
                new Pose3d(
                5.987542000,
                -0.003810000,
                1.301750000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                17,
                new Pose3d(
                4.073906000,
                3.306318000,
                0.308102000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                18,
                new Pose3d(
                3.657600000,
                4.025900000,
                0.308102000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                19,
                new Pose3d(
                4.073906000,
                4.745482000,
                0.308102000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                20,
                new Pose3d(
                4.904740000,
                4.745482000,
                0.308102000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                21,
                new Pose3d(
                5.321046000,
                4.025900000,
                0.308102000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            ),
            new AprilTag(
                22,
                new Pose3d(
                4.904740000,
                3.306318000,
                0.308102000,
                new Rotation3d(
                    new Quaternion(0.0, 0.0, 0.0, 1.0))
                )
            )
        ),
        17.548225,
        8.0518
      );    // Constants for camera position relative to robot center
    private static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
        // Modify these values based on your robot's camera mounting position
        0.5, 0.0, 0.5,  // x, y, z in meters
        new Rotation3d(0, 0, 0)  // roll, pitch, yaw in radians
    );

    public VisionSubsystem() {
        // Initialize the camera with its network table name
        camera = new PhotonCamera("1");

        // Create pose estimator
        photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            CAMERA_TO_ROBOT
        );
        tab.add("Field", fieldVisualization);
    }

    @Override
    public void periodic() {
        // Get the latest pipeline result
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : results){
            if (result.hasTargets()) {
                NetworkTableInstance.getDefault().getTable("Vision").getEntry("hasTargets").setBoolean(true);
                // Get estimated robot pose
                Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(result);
                
                if (estimatedPose.isPresent()) {
                    // Update the current pose
                    currentPose = estimatedPose.get().estimatedPose.toPose2d();
                    NetworkTableInstance.getDefault().getTable("Vision").getEntry("RobotX").setDouble(currentPose.getX());
                    NetworkTableInstance.getDefault().getTable("Vision").getEntry("RobotY").setDouble(currentPose.getY());
                    // You might want to send this to other subsystems or log it
                    fieldVisualization.setRobotPose(currentPose);
                }
            }
            else {
                NetworkTableInstance.getDefault().getTable("Vision").getEntry("hasTargets").setBoolean(false);
            }
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
}