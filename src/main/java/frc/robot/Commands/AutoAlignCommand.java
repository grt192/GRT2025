package frc.robot.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;


import java.nio.file.Path;

import com.ctre.phoenix6.swerve.SwerveRequest.NativeSwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;



public class AutoAlignCommand {

    static Pose2d reefPose = new Pose2d(3.607, 5.312, Rotation2d.fromDegrees(-55));
    
    static PathPlannerPath getAlignPath;
    private static String reefName = "reefAlignPath";
    private static String sourceName = "sourceAlignPath";
    
    private static Command runAlignPath;
    
    static PathConstraints constraints = new PathConstraints(
        4.6,
        3,
        Units.degreesToRadians(540), 
        Units.degreesToRadians(720)
    );

    private static PathPlannerPath getAlignPath(String path){
        try {
            getAlignPath = PathPlannerPath.fromPathFile(path);
        } catch (Exception e) {
            e.printStackTrace();
            // Handle exception as needed, maybe use default values or fallback
        }
        return getAlignPath;
    }

    public static Command runAlignPath (SwerveSubsystem swerveSubsystem, String pathName){
        PathPlannerPath path = getAlignPath(pathName);
        // runAlignPath.addRequirements(swerveSubsystem);
        runAlignPath = AutoBuilder.pathfindThenFollowPath(
            path,
            constraints
        );
        runAlignPath.addRequirements(swerveSubsystem); 
        return runAlignPath;
    }
    
    public static Command reefTest(SwerveSubsystem swerveSubsystem){
        PathPlannerPath path = getAlignPath(reefName);
        // runAlignPath.addRequirements(swerveSubsystem); 
        runAlignPath = AutoBuilder.pathfindThenFollowPath(
            path,
            constraints
        );
        runAlignPath.addRequirements(swerveSubsystem); 
        return runAlignPath;
        // return runAlignPath(swerveSubsystem, reefName);
    }

    public static Command sourceTest(SwerveSubsystem swerveSubsystem){
        PathPlannerPath path = getAlignPath(sourceName);
        runAlignPath = AutoBuilder.pathfindThenFollowPath(
            path,
            constraints
        );
        runAlignPath.addRequirements(swerveSubsystem);
        return runAlignPath; 
        // return runAlignPath(swerveSubsystem, sourceName);
    }

    // public static Command closeReef(SwerveSubsystem swerveSubsystem, Pose2d currentPose){
    //     Translation2d currentTrans = currentPose.getTranslation();
    //     currentTrans.

    // }

}

