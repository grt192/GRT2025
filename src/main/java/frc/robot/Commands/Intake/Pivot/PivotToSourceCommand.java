package frc.robot.Commands.Intake.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstans.PivotConstants;
import frc.robot.subsystems.Intake.Pivot.PivotSubsystem;

public class PivotToSourceCommand extends Command{
   private final PivotSubsystem pivotSubsystem; 

   public PivotToSourceCommand(PivotSubsystem pivotSubsystem){
       this.pivotSubsystem = pivotSubsystem;
       this.addRequirements(pivotSubsystem);
   }

   @Override
   public void initialize(){
       pivotSubsystem.setPositionReference(PivotConstants.SOURCE_POS);
   }

   @Override
   public boolean isFinished(){
        return Math.abs(pivotSubsystem.getClosedLoopError())
            < PivotConstants.PIVOT_TOLERANCE;
   }
}