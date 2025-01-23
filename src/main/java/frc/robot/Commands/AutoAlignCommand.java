package frc.robot.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;


import java.nio.file.Path;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;



public class AutoAlignCommand {

    static Pose2d reefPose = new Pose2d(1.05, -0.74, Rotation2d.fromDegrees(54));
    
    static PathPlannerPath getAlignPath;
    private static String reefName = "testReef";
    
    
    static PathConstraints constraints = new PathConstraints(
        3.227,
            3,
        Units.degreesToRadians(540), 
        Units.degreesToRadians(720)
    );

    // private static PathPlannerPath getAlignPath(String path){
    //     try {
    //         getAlignPath = PathPlannerPath.fromPathFile(path);
    //     } catch (Exception e) {
    //         e.printStackTrace();
    //         // Handle exception as needed, maybe use default values or fallback
    //     }
    //     return getAlignPath;
    // }

    // public static Command alignToPose(SwerveSubsystem swerveSubsystem, Pose2d targetPose){
    //     Command alignToPose = AutoBuilder.pathfindToPose(
    //         targetPose,
    //         constraints
    //         );
    
    //     alignToPose.addRequirements(swerveSubsystem);

    //     return alignToPose; 
    // }

    // public static Command runAlignPath (SwerveSubsystem swerveSubsystem, String pathName){
    //     PathPlannerPath path = getAlignPath(pathName);
    //     Command runAlignPath = AutoBuilder.pathfindThenFollowPath(
    //         path,
    //         constraints
    //     );
    //     runAlignPath.addRequirements(swerveSubsystem); 
    //     return runAlignPath;
    // }
    
//     public static SequentialCommandGroup reefTest(SwerveSubsystem swerveSubsystem){
//         return new SequentialCommandGroup(
//         //  alignToPose(swerveSubsystem, reefPose)
//         runAlignPath(swerveSubsystem, reefName)
//         );
//     }
     }


