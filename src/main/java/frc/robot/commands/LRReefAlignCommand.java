package frc.robot.commands;

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
import frc.robot.Constants.IntakeConstants.ReefAlignConstants;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.IntakeConstants.AligningConstants;

public class LRReefAlignCommand extends Command{
    static SwerveSubsystem swerveSubsystem;
    static FieldManagementSubsystem fmsSubsystem;
    static AlignUtil alignSubsystem;
    static List<Pose2d> currentRightPoseList;
    static List<Pose2d> currentLeftPoseList;
    Boolean isRight;
    static PathPlannerPath getAlignPath;
    private static String followPath;
    
    public LRReefAlignCommand(SwerveSubsystem swerveSubsystem, FieldManagementSubsystem fmsSubsystem, Boolean isRight) {
        this.swerveSubsystem = swerveSubsystem;
        this.fmsSubsystem = fmsSubsystem;
        alignSubsystem = new AlignUtil(swerveSubsystem, swerveSubsystem.getRobotPosition());
        this.isRight = isRight;
    }

    @Override
    public void initialize() {
        if (fmsSubsystem.isRedAlliance()) {
            currentRightPoseList = ReefAlignConstants.redRightReefPoseList;
            currentLeftPoseList = ReefAlignConstants.redLeftReefPoseList;
        }
        else {
            currentRightPoseList = ReefAlignConstants.blueRightReefPoseList;
            currentLeftPoseList = ReefAlignConstants.blueLeftReefPoseList;
        }
        if (isRight){
            Pose2d closestRight = swerveSubsystem.getRobotPosition().nearest(currentRightPoseList);
            int index = currentRightPoseList.indexOf(closestRight);
            followPath = ReefAlignConstants.reefPathList.get(2 * index + 1);
        }
        else {
            Pose2d closestLeft = swerveSubsystem.getRobotPosition().nearest(currentLeftPoseList);
            // System.out.println(swerveSubsystem.getRobotPosition());
            int index = currentLeftPoseList.indexOf(closestLeft);
            followPath = ReefAlignConstants.reefPathList.get(2 * index);
        }
        System.out.println("InITing");
        alignSubsystem.runAlignPath(followPath).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.getCurrentCommand().cancel();
    }

}
