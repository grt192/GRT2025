package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.pivot.PivotState;
import frc.robot.subsystems.Intake.pivot.PivotSubsystem;

public class SetPivotOuttakeCommand extends Command{
    private PivotSubsystem pivotSubsystem;

    public SetPivotOuttakeCommand(PivotSubsystem pivotSubsystem) {
        this.addRequirements(pivotSubsystem);
        this.pivotSubsystem = pivotSubsystem;
    }

    @Override
    public void execute() {
        this.pivotSubsystem.setState(PivotState.OUTTAKE);
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.atState(PivotState.OUTTAKE);
    }
}
