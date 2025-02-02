package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.pivot.PivotState;
import frc.robot.subsystems.Intake.pivot.PivotSubsystem;

public class SetPivotSourceCommand extends Command{
    private PivotSubsystem pivotSubsystem;

    public SetPivotSourceCommand(PivotSubsystem pivotSubsystem) {
        this.addRequirements(pivotSubsystem);
        this.pivotSubsystem = pivotSubsystem;
    }

    @Override
    public void execute() {
        this.pivotSubsystem.setState(PivotState.SOURCE);
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.atState(PivotState.SOURCE);
    }
}
