package frc.robot.Commands.Intake.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstans.PivotConstants;
import frc.robot.subsystems.Intake.Pivot.PivotSubsystem;

public class PivotToOuttakeCommand extends Command{
   private final PivotSubsystem pivotSubsystem; 

   public PivotToOuttakeCommand(PivotSubsystem pivotSubsystem){
       this.pivotSubsystem = pivotSubsystem;
       this.addRequirements(pivotSubsystem);
   }

   @Override
   public void initialize(){
    // if(pivotSubsystem.getLimitSwitch()){pivotSubsystem.setEncoderZero();}
       pivotSubsystem.setPositionReferenceWithVoltage(PivotConstants.OUTTAKE_POS);
   }

   @Override
   public boolean isFinished(){
        return Math.abs(pivotSubsystem.getClosedLoopError())
            < PivotConstants.PIVOT_TOLERANCE;
   }
}