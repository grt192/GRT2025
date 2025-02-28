package frc.robot.Commands.Intake.Roller;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstans.RollerConstants;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;

public class RollerOutCommand extends Command{
   
    private final RollerSubsystem rollerSubsystem;

    public RollerOutCommand(RollerSubsystem rollerSubsystem){
        this.rollerSubsystem = rollerSubsystem;
        this.addRequirements(rollerSubsystem);
    }

    @Override
    public void initialize(){
        rollerSubsystem.setRollerSpeed(RollerConstants.ROLLER_OUT_SPEED);
    }

    @Override
    public boolean isFinished(){
        return Math.abs(rollerSubsystem.getClosedLoopError())
            < RollerConstants.ROLLER_TOLERANCE;
    }
}
