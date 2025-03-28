
package frc.robot.Commands.Align;

import java.util.List;

import org.opencv.photo.AlignExposures;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FieldManagementSubsystem.FieldManagementSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.AlignUtil;
import frc.robot.Constants.AlignConstants;

public class SourceAlignCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final FieldManagementSubsystem fmsSubsystem;
    private final AlignUtil alignSubsystem;
    static String followPath;
    static List<Pose2d> currentPoseList;
    static Pose2d currentPose;
    

    public SourceAlignCommand(SwerveSubsystem swerveSubsystem, FieldManagementSubsystem fmsSubsystem){
        this.swerveSubsystem = swerveSubsystem;
        this.fmsSubsystem = fmsSubsystem;
        this.alignSubsystem = new AlignUtil(swerveSubsystem, swerveSubsystem.getRobotPosition());

    }


   @Override
   public void initialize() {
    if (fmsSubsystem.isRedAlliance()){
        currentPoseList = AlignConstants.redSourcePoses;
    }
    else {
        currentPoseList = AlignConstants.blueSourcePoses;
    }
    currentPose = swerveSubsystem.getRobotPosition().nearest(currentPoseList);
    int index = currentPoseList.indexOf(currentPose);
    followPath = AlignConstants.sourcePathList.get(index);
    System.out.println(followPath);
    alignSubsystem.runAlignPath(followPath, swerveSubsystem.getRobotPosition()).schedule();
   }

   @Override
   public void end(boolean interrupted) {
       swerveSubsystem.getCurrentCommand().cancel();
   }

}
