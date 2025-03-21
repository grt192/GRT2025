    
package frc.robot.util;

import java.util.List;

import javax.print.attribute.standard.PresentationDirection;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Constants.AlignConstants;
import frc.robot.Commands.Swerve.DriveBackwardsCommand;

public class AlignUtil {

    private static SwerveSubsystem swerveSubsystem;
    private final Pose2d currentPosition;
    static PathPlannerPath getAlignPath;
    private static PathConstraints constraints;
    static Command runAlignPath;

    public AlignUtil(SwerveSubsystem swerveSubsystem, Pose2d currentPosition){
        this.swerveSubsystem = swerveSubsystem;
        this.currentPosition = currentPosition;

        this.constraints = new PathConstraints(
            4.6,
            3,
            Units.degreesToRadians(540), 
            Units.degreesToRadians(720)
        );

    }
   
    /**
     * Uses getAlignPath to get the pathplanner path and follows it
     * @param swerveSubsystem
     * @param pathName name of the path
     */

    public Command runAlignPath (String pathName, Pose2d currentPos, Boolean onFly) {
        Translation2d currentTrans = swerveSubsystem.getRobotPosition().getTranslation();
        Translation2d pathStartTrans = getAlignPath(pathName).getStartingHolonomicPose().get().getTranslation();

        // if (currentTrans.getDistance(pathStartTrans) <= AlignConstants.distanceTolerance) {
        //     int index = AlignConstants.reefPathList.indexOf(pathName) / 2;
        //     ChassisSpeeds drivePower = AlignConstants.reefdirectionList.get(index);

        //     PathPlannerPath path = getAlignPath(pathName);
        //     if (path == null) {
        //         System.out.println("NOOO00000000000000O");
        //         return Commands.none();

        //     }
        //     Command alignPath = AutoBuilder.pathfindThenFollowPath(
        //         path,
        //         constraints);

        //     System.out.println("XXXXXXXXXXXXXXXXXX");
        //     alignPath.addRequirements(swerveSubsystem);

        //     runAlignPath = (Command) new SequentialCommandGroup(
        //         new DriveBackwardsCommand(swerveSubsystem, drivePower).until(
        //             () -> swerveSubsystem.getRobotPosition().getTranslation()
        //             .getDistance(pathStartTrans) > AlignConstants.distanceTolerance),
        //             alignPath
        //     );
            
        // }
        // else {
        //     PathPlannerPath path = getAlignPath(pathName);
        //     if (path == null) {
        //         System.out.println("NOOOOOOOOPOOOOOOO");
        //         return Commands.none();
        //     }

        //     System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAA");
        //     runAlignPath = AutoBuilder.pathfindThenFollowPath(
        //         path,
        //         constraints);
        // }

        //TESTING PATHFIND TO PATH VS ON THE FLY PATH 
        //ADD BOOLEAN onFLY 
        if (onFly) {
            PathPlannerPath unusedPath = getAlignPath(pathName);
            PathPlannerPath path = getAlignPath(
                unusedPath.getWaypoints(), 
                unusedPath.getGoalEndState()
            );

            runAlignPath = AutoBuilder.followPath(path);
        }
        else {
            PathPlannerPath path = getAlignPath(pathName);
            runAlignPath = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints
            );
        }

        return runAlignPath; 

    }

        /**
     * takes the path name and returns the PathPlanner Path 
     * @param pathName
     * @return path file
     */
    public static PathPlannerPath getAlignPath(String pathName) {
        try {
            getAlignPath = PathPlannerPath.fromPathFile(pathName);
        } catch (Exception e) {
            e.printStackTrace();
            // Handle exception as needed, maybe use default values or fallback
        }
        return getAlignPath;
    }

    public PathPlannerPath getAlignPath (List<Waypoint> pathWaypoints, GoalEndState goalEndState){

        PathPlannerPath getAlignPath = new PathPlannerPath (
            pathWaypoints,
            constraints,
            null, 
            goalEndState
        );
        return getAlignPath;
    }


}

