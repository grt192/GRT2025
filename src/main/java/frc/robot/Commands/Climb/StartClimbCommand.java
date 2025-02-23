package frc.robot.Commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimbConstants;
import frc.robot.subsystems.ClimbSubsystem;

public class StartClimbCommand extends Command{
    private final ClimbSubsystem climbSubsystem;
    
    public StartClimbCommand(ClimbSubsystem climbSubsystem){
        this.climbSubsystem = climbSubsystem;
        this.addRequirements(climbSubsystem);
    }

    @Override
    public void initialize(){
        climbSubsystem.start();
    }

    @Override
    public boolean isFinished(){
        return climbSubsystem.getClosedLoopError() 
            < ClimbConstants.CLIMB_TOLLERANCE;
    }
}