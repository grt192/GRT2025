package frc.robot.commands;

import java.util.List;

import com.fasterxml.jackson.databind.JsonSerializable.Base;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.controllers.BaseDriveController;
import frc.robot.subsystems.FieldManagementSubsystem.FieldManagementSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class LRReefAlignCommand extends Command{
    private final SwerveSubsystem swerveSubsystem;
    private final FieldManagementSubsystem fmsSubsystem;
    private List<Pose2d> currentRightPoseList;
    private List<Pose2d> currentLeftPoseList;
    Boolean isRight;
    static PathPlannerPath getAlignPath;

    private static Command runAlignPath;

    private static String reefName = "reefAlignPath";
    private static String sourceName = "sourceAlignPath";
    private static String A_alignName = "A align";
    private static String B_alignName = "B align";
    private static String C_alignName = "C align";
    private static String D_alignName = "D align";
    private static String E_alignName = "E align";
    private static String F_alignName = "F align";
    private static String G_alignName = "G align";
    private static String H_alignName = "H align";
    private static String I_alignName = "I align";
    private static String J_alignName = "J align";
    private static String K_alignName = "K align";
    private static String L_alignName = "L align";
    private static String followPath;


    final List<Pose2d> blueLeftReefPoseList = List.of(
        getAlignPath(A_alignName).getStartingHolonomicPose().get(),
        getAlignPath(C_alignName).getStartingHolonomicPose().get(),
        getAlignPath(E_alignName).getStartingHolonomicPose().get(),
        getAlignPath(G_alignName).getStartingHolonomicPose().get(),
        getAlignPath(I_alignName).getStartingHolonomicPose().get(),
        getAlignPath(K_alignName).getStartingHolonomicPose().get()
    );
    
    final List<Pose2d> blueRightReefPoseList = List.of(
        getAlignPath(B_alignName).getStartingHolonomicPose().get(),
        getAlignPath(D_alignName).getStartingHolonomicPose().get(),
        getAlignPath(F_alignName).getStartingHolonomicPose().get(),
        getAlignPath(H_alignName).getStartingHolonomicPose().get(),
        getAlignPath(J_alignName).getStartingHolonomicPose().get(),
        getAlignPath(L_alignName).getStartingHolonomicPose().get()
    ); 

    
    final List<Pose2d> redLeftReefPoseList = List.of(
        getAlignPath(A_alignName).flipPath().getStartingHolonomicPose().get(),
        getAlignPath(C_alignName).flipPath().getStartingHolonomicPose().get(),
        getAlignPath(E_alignName).flipPath().getStartingHolonomicPose().get(),
        getAlignPath(G_alignName).flipPath().getStartingHolonomicPose().get(),
        getAlignPath(I_alignName).flipPath().getStartingHolonomicPose().get(),
        getAlignPath(K_alignName).flipPath().getStartingHolonomicPose().get()
    );
    
    final List<Pose2d> redRightReefPoseList = List.of(
        getAlignPath(B_alignName).flipPath().getStartingHolonomicPose().get(),
        getAlignPath(D_alignName).flipPath().getStartingHolonomicPose().get(),
        getAlignPath(F_alignName).flipPath().getStartingHolonomicPose().get(),
        getAlignPath(H_alignName).flipPath().getStartingHolonomicPose().get(),
        getAlignPath(J_alignName).flipPath().getStartingHolonomicPose().get(),
        getAlignPath(L_alignName).flipPath().getStartingHolonomicPose().get()
    ); 

    //put the left on top of right
    static List<String> reefPathList = List.of(
        A_alignName,
        B_alignName,
        C_alignName,
        D_alignName,
        E_alignName,
        F_alignName,
        G_alignName,
        H_alignName,
        I_alignName,
        J_alignName,
        K_alignName,
        L_alignName
    );
    
    static PathConstraints constraints = new PathConstraints(
        4.6,
        3,
        Units.degreesToRadians(540), 
        Units.degreesToRadians(720)
    );

    public LRReefAlignCommand(SwerveSubsystem swerveSubsystem, FieldManagementSubsystem fmsSubsystem, Boolean isRight) {
        this.swerveSubsystem = swerveSubsystem;
        this.fmsSubsystem = fmsSubsystem;
        this.isRight = isRight;
        // this.addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        if (fmsSubsystem.isRedAlliance()) {
            currentRightPoseList = redRightReefPoseList;
            currentLeftPoseList = redLeftReefPoseList;
        }
        else {
            currentRightPoseList = blueRightReefPoseList;
            currentLeftPoseList = blueLeftReefPoseList;
        }
        if (isRight) {
            Pose2d closestRight = swerveSubsystem.getRobotPosition().nearest(currentRightPoseList);
            int index = currentRightPoseList.indexOf(closestRight);
            followPath = reefPathList.get(2 * index + 1);
        }
        else {
            Pose2d closestLeft = swerveSubsystem.getRobotPosition().nearest(currentLeftPoseList);
            // System.out.println(swerveSubsystem.getRobotPosition());
            int index = currentLeftPoseList.indexOf(closestLeft);
            followPath = reefPathList.get(2 * index);
        }
        System.out.println("InITing");

        runAlignPath(followPath).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        if (swerveSubsystem.getCurrentCommand() != null) {
            swerveSubsystem.getCurrentCommand().cancel();
        }
    }

    // @Override
    // public boolean isFinished() {
    //     return driveController.getForwardPower() <= 0.05 && (driveController.getLeftPower() <= 0.05);
    // }

    /**
     * Uses getAlignPath to get the pathplanner path and follows it
     * @param swerveSubsystem
     * @param pathName name of the path
     */
    public Command runAlignPath (String pathName) {
        PathPlannerPath path = getAlignPath(pathName);
        if (path == null) {
            this.cancel();
        }
        System.out.print(pathName);

        runAlignPath = AutoBuilder.pathfindThenFollowPath(
            path,
            constraints
        );

        runAlignPath.addRequirements(swerveSubsystem); 
        System.out.println("RUNNING");
        return runAlignPath;
    }

        /**
     * takes the path name and returns the PathPlanner Path 
     * @param pathName
     * @return path file
     */
    private PathPlannerPath getAlignPath(String pathName) {
        try {
            getAlignPath = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            e.printStackTrace();
            // Handle exception as needed, maybe use default values or fallback
        }
        return getAlignPath;
    }
}
