package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Constants.IntakeConstants.AligningConstants;

public class AlignSubsystem extends SubsystemBase{

    private static SwerveSubsystem swerveSubsystem;
    static PathPlannerPath getAlignPath;
    private static PathConstraints constraints;
    static Command runAlignPath;

    public AlignSubsystem(){

        PathConstraints constraints = new PathConstraints(
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

    public Command runAlignPath (String pathName) {
        Translation2d currentTrans = swerveSubsystem.getRobotPosition().getTranslation();
        Translation2d pathStartTrans = getAlignPath("pathName").getStartingHolonomicPose().get().getTranslation();

        if (currentTrans.getDistance(pathStartTrans)>AligningConstants.distanceTolerance){
            PathPlannerPath path = getAlignPath(pathName);
            if (path == null) {
                return Commands.none();
            }
            //System.out.print(pathName);
            runAlignPath = AutoBuilder.pathfindThenFollowPath(
                path,
                constraints
            );
    
            runAlignPath.addRequirements(swerveSubsystem); 
           // System.out.println("RUNNING");
            return runAlignPath;
        }
        else {
            PathPlannerPath path = getAlignPath(pathName);
            List<Waypoint> pathWaypoints = path.getWaypoints();
            GoalEndState goalEndState = path.getGoalEndState();
            PathPlannerPath onTheFlyPath = getAlignPath(pathWaypoints, goalEndState);
            runAlignPath = AutoBuilder.followPath(onTheFlyPath);

            return runAlignPath;
        }

    }

        /**
     * takes the path name and returns the PathPlanner Path 
     * @param pathName
     * @return path file
     */
    public PathPlannerPath getAlignPath(String pathName) {
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