package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class LRReefAlignCommand extends Command{
    static SwerveSubsystem swerveSubsystem;
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


    static List<Pose2d> leftReefPoseList = List.of(
        getAlignPath(A_alignName).getStartingHolonomicPose().get(),
        getAlignPath(C_alignName).getStartingHolonomicPose().get(),
        getAlignPath(E_alignName).getStartingHolonomicPose().get(),
        getAlignPath(G_alignName).getStartingHolonomicPose().get(),
        getAlignPath(I_alignName).getStartingHolonomicPose().get(),
        getAlignPath(K_alignName).getStartingHolonomicPose().get()
    );
    
    static List<Pose2d> rightReefPoseList = List.of(
        getAlignPath(B_alignName).getStartingHolonomicPose().get(),
        getAlignPath(D_alignName).getStartingHolonomicPose().get(),
        getAlignPath(F_alignName).getStartingHolonomicPose().get(),
        getAlignPath(H_alignName).getStartingHolonomicPose().get(),
        getAlignPath(J_alignName).getStartingHolonomicPose().get(),
        getAlignPath(L_alignName).getStartingHolonomicPose().get()
    ); 
    
    //put the left on top of right
    static List<Pose2d> allReefPoseList = List.of(
        getAlignPath(A_alignName).getStartingHolonomicPose().get(),
        getAlignPath(B_alignName).getStartingHolonomicPose().get(),
        getAlignPath(C_alignName).getStartingHolonomicPose().get(),
        getAlignPath(D_alignName).getStartingHolonomicPose().get(),
        getAlignPath(E_alignName).getStartingHolonomicPose().get(),
        getAlignPath(F_alignName).getStartingHolonomicPose().get(),
        getAlignPath(G_alignName).getStartingHolonomicPose().get(),
        getAlignPath(H_alignName).getStartingHolonomicPose().get(),
        getAlignPath(I_alignName).getStartingHolonomicPose().get(),
        getAlignPath(J_alignName).getStartingHolonomicPose().get(),
        getAlignPath(K_alignName).getStartingHolonomicPose().get(),
        getAlignPath(L_alignName).getStartingHolonomicPose().get()
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

    public LRReefAlignCommand(SwerveSubsystem swerveSubsystem, Boolean isRight) {
        this.swerveSubsystem = swerveSubsystem;
        this.isRight = isRight;
    }

    @Override
    public void initialize() {
        if (isRight){
            Pose2d closestRight = swerveSubsystem.getRobotPosition().nearest(rightReefPoseList);
            int index = rightReefPoseList.indexOf(closestRight);
            followPath = reefPathList.get(2 * index + 1);
        }
        else {
            Pose2d closestLeft = swerveSubsystem.getRobotPosition().nearest(leftReefPoseList);
            System.out.println(swerveSubsystem.getRobotPosition());
            int index = leftReefPoseList.indexOf(closestLeft);
            followPath = reefPathList.get(2 * index);
        }
        runAlignPath(followPath).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.getCurrentCommand().cancel();
    }

        /**
     * Uses getAlignPath to get the pathplanner path and follows it
     * @param swerveSubsystem
     * @param pathName name of the path
     */
    public static Command runAlignPath (String pathName) {
        PathPlannerPath path = getAlignPath(pathName);
        System.out.print(path);

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
    private static PathPlannerPath getAlignPath(String pathName) {
        try {
            getAlignPath = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            e.printStackTrace();
            // Handle exception as needed, maybe use default values or fallback
        }
        return getAlignPath;
    }
}
