package frc.robot.commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;




public class AutoAlignCommand {

    //static Pose2d reefTestPose = new Pose2d(3.607, 5.312, Rotation2d.fromDegrees(-55));
    private static Pose2d currentPose;
    
    static PathPlannerPath getAlignPath;

    private static String reefName = "reefAlignPath";
    private static String sourceName = "sourceAlignPath";

    static List<Pose2d> ReefPoseList = List.of(
        getAlignPath(reefName).getStartingHolonomicPose().get()
    ); 
    
    static List<String> ReefPathList = List.of(
        reefName
    );

    private static Command runAlignPath;
    
    static PathConstraints constraints = new PathConstraints(
        4.6,
        3,
        Units.degreesToRadians(540), 
        Units.degreesToRadians(720)
    );

    //Gets the path from pathplanner using the path name
    private static PathPlannerPath getAlignPath(String path){
        try {
            getAlignPath = PathPlannerPath.fromPathFile(path);
        } catch (Exception e) {
            e.printStackTrace();
            // Handle exception as needed, maybe use default values or fallback
        }
        return getAlignPath;
    }

    //runs the path from getAlignPath
    public static Command runAlignPath (SwerveSubsystem swerveSubsystem, String pathName){
        PathPlannerPath path = getAlignPath(pathName);

        runAlignPath = AutoBuilder.pathfindThenFollowPath(
            path,
            constraints
        );

        runAlignPath.addRequirements(swerveSubsystem); 
        return runAlignPath;
    }
    
    //aligns to the reef L/K on test field
    public static Command reefTest(SwerveSubsystem swerveSubsystem){

        PathPlannerPath path = getAlignPath(reefName);
        
        runAlignPath = AutoBuilder.pathfindThenFollowPath(
            path,
            constraints
        );

        return runAlignPath(swerveSubsystem, reefName);
    }

    //aligns to top reef on test field
    public static Command sourceTest(SwerveSubsystem swerveSubsystem){

        PathPlannerPath path = getAlignPath(sourceName);
        runAlignPath = AutoBuilder.pathfindThenFollowPath(
            path,
            constraints
        );

        return runAlignPath(swerveSubsystem, sourceName);
    }

    //aligns to the closest reef pose from the ReefPoseList
    public static Command closeReefAlign(SwerveSubsystem swerveSubsystem){

        currentPose = swerveSubsystem.getRobotPosition();
        Pose2d closestReef = currentPose.nearest(ReefPoseList);
        int index = ReefPoseList.indexOf(closestReef);
        String closestPath =  ReefPathList.get(index); 

        return runAlignPath(swerveSubsystem, closestPath);   
    }

}

