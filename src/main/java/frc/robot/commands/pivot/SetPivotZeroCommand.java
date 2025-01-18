package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.subsystems.pivot.PivotSubsystem;

public class SetPivotZeroCommand extends Command{
    private PivotSubsystem pivotSubsystem;

    public SetPivotZeroCommand(PivotSubsystem pivotSubsystem) {
        this.addRequirements(pivotSubsystem);
        this.pivotSubsystem = pivotSubsystem;
    }

    @Override
    public void execute() {
        this.pivotSubsystem.setState(PivotState.ZERO);
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.atState(PivotState.ZERO);
    }
}
