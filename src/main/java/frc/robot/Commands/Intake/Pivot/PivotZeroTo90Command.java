package frc.robot.Commands.Intake.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstans.PivotConstants;
import frc.robot.subsystems.Intake.Pivot.PivotSubsystem;

public class PivotZeroTo90Command extends Command{
   private final PivotSubsystem pivotSubsystem; 

   public PivotZeroTo90Command(PivotSubsystem pivotSubsystem){
       this.pivotSubsystem = pivotSubsystem;
       this.addRequirements(pivotSubsystem);
   }

   @Override
   public void initialize(){
       pivotSubsystem.setPosition(Math.PI / 2.);
   }

   @Override
   public boolean isFinished(){
    return true;
   }
}