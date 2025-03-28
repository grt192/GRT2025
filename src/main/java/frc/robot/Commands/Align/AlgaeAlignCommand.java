 
package frc.robot.Commands.Align;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.FieldManagementSubsystem.FieldManagementSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.AlignUtil;
import frc.robot.Constants.AlignConstants;
import edu.wpi.first.math.geometry.Translation2d;

public class AlgaeAlignCommand extends Command{
    static SwerveSubsystem swerveSubsystem;
    static FieldManagementSubsystem fmsSubsystem;
    static AlignUtil alignSubsystem;
    static List<Pose2d> currentPoseList;
    static PathPlannerPath getAlignPath;
    private static String followPath;
    
    public AlgaeAlignCommand (SwerveSubsystem swerveSubsystem, FieldManagementSubsystem fmsSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.fmsSubsystem = fmsSubsystem;
        alignSubsystem = new AlignUtil(swerveSubsystem, swerveSubsystem.getRobotPosition());
    }

    @Override
    public void initialize() {
        if (fmsSubsystem.isRedAlliance()) {
            currentPoseList = AlignConstants.redAlgaeAlignPoses;
        }
        else {
            currentPoseList = AlignConstants.blueAlgaeAlignPoses;
        }
        
        Pose2d closestPose = swerveSubsystem.getRobotPosition().nearest(currentPoseList);
        int index  = currentPoseList.indexOf(closestPose);
        followPath = AlignConstants.algaeAlignNames.get(index);

        System.out.println("InITing");
        alignSubsystem.runAlignPath(followPath, swerveSubsystem.getRobotPosition()).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        if (swerveSubsystem.getCurrentCommand() != null) {
            swerveSubsystem.getCurrentCommand().cancel();
        }
    }

}
