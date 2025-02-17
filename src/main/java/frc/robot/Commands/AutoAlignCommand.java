package frc.robot.commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;




public class AutoAlignCommand {

    // static Pose2d reefTestPose = new Pose2d(3.607, 5.312, Rotation2d.fromDegrees(-55));
    private static Pose2d currentPose = new Pose2d(3.473, 5.483, Rotation2d.fromDegrees(-57));

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

    /**
     * Uses getAlignPath to get the pathplanner path and follows it
     * @param swerveSubsystem
     * @param pathName name of the path
     */
    public static Command runAlignPath (SwerveSubsystem swerveSubsystem, String pathName) {
        PathPlannerPath path = getAlignPath(pathName);
        System.out.print(pathName);

        runAlignPath = AutoBuilder.pathfindThenFollowPath(
            path,
            constraints
        );

        runAlignPath.addRequirements(swerveSubsystem); 
        return runAlignPath;
    }
    
    /**
     * TEST METHOD - Uses runAlignPath to align to the test reef station (reef L/K)
     * @param swerveSubsystem
     */
    public static Command reefTest(SwerveSubsystem swerveSubsystem){
      return runAlignPath(swerveSubsystem, reefName);
    }

    /**
     * TEST METHOD - Uses runAlignPath to align to the test source (source at the top of the field)
     * @param swerveSubsystem
     */
    public static Command sourceTest(SwerveSubsystem swerveSubsystem){
       
        return runAlignPath(swerveSubsystem, sourceName);
    }

    public static Command lrReefAlign(SwerveSubsystem swerveSubsystem, Boolean Right) {
        if (Right){
            Pose2d closestRight = currentPose.nearest(rightReefPoseList);
            int index = rightReefPoseList.indexOf(closestRight);
            followPath =reefPathList.get(2*index +1);
        }
        else {
            Pose2d closestLeft = currentPose.nearest(leftReefPoseList);
            int index = rightReefPoseList.indexOf(closestLeft);
            followPath = reefPathList.get(2*index);
        }
        return runAlignPath(swerveSubsystem, followPath);
    }
    /**
     * Aligns to the closest reef pose unless boolean switchPose is true then it aligns to the pose next to it
     * @param swerveSubsystem
     * @param switchPose boolean that decides whether to align to the closest pose or the one next to it (on the same side of the hexagon)
     */
    public static Command closeReefAlign(SwerveSubsystem swerveSubsystem, Boolean switchPose) {

        Pose2d closestReef = currentPose.nearest(allReefPoseList);
        //align to closest pose
        if(switchPose == false) {
            int index = allReefPoseList.indexOf(closestReef);
            followPath = reefPathList.get(index);
        }
        //align to the pose next to the closest pose 
        else {
            // if the closest pose is on the right, find the index of it in the list of right poses and get the matching left path using algebraic term 
            if (rightReefPoseList.contains(closestReef)) {
                int index = rightReefPoseList.indexOf(closestReef);
                followPath = reefPathList.get( 2*index );
            }
            // if the closest pose is on the left, find the index and get the matching right path
            else {
                int index = leftReefPoseList.indexOf(closestReef);
                followPath = reefPathList.get( 2*index+1 );
            
            }

        }
        return runAlignPath(swerveSubsystem, followPath);
    }

    public static void closeReefMath(SwerveSubsystem swerveSubsystem, Boolean switchPose) {

        System.out.println("switch pose is " + switchPose);

        Pose2d closestReef = currentPose.nearest(allReefPoseList);
        //align to closest pose
        if(switchPose == false) {
            int index = allReefPoseList.indexOf(closestReef);
            followPath = reefPathList.get(index);
         
        }
        //align to the pose next to the closest pose 
        else {
            // if the closest pose is on the right, find the index of it in the list of right poses and get the matching left path using algebraic term 
            if (rightReefPoseList.contains(closestReef)) {
                int index = rightReefPoseList.indexOf(closestReef);
                followPath = reefPathList.get( 2*index );
                System.out.println("Closest to right");
               
            }
            // if the closest pose is on the left, find the index and get the matching right path
            else {
                int index = leftReefPoseList.indexOf(closestReef);
                followPath = reefPathList.get( 2*index+1 );
                System.out.println("Closest to left");
            }

        }
        System.out.println("CLOSE PATH"+ followPath);

       
    }


}

