package frc.robot.subsystems.Vision;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Transform3d;

public class CameraConfig {
    private String cameraName;
    private Transform3d cameraPose;
    private PoseStrategy poseStrategy;
    
    public CameraConfig(
        String cameraName, Transform3d cameraPose, PoseStrategy poseStrategy
    ){
        this.cameraName = cameraName;
        this.cameraPose = cameraPose;
        this.poseStrategy = poseStrategy;
    }

    public String getCameraName(){
        return cameraName;
    }

    public Transform3d getCameraPose(){
        return cameraPose;
    }

    public PoseStrategy getPoseStrategy(){
        return poseStrategy;
    }
}
