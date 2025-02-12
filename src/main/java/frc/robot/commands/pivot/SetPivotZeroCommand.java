package frc.robot.commands.Pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.pivot.PivotState;
import frc.robot.subsystems.Intake.pivot.PivotSubsystem;

public class SetPivotZeroCommand extends Command{
    private PivotSubsystem pivotSubsystem;

    public SetPivotZeroCommand(PivotSubsystem pivotSubsystem) {
        this.addRequirements(pivotSubsystem);
        this.pivotSubsystem = pivotSubsystem;
    }

    @Override
    public void execute() {
        this.pivotSubsystem.setState(PivotState.ZERO);
        // System.out.println("horizontal" + pivotSubsystem.getPositionError());
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.atState(PivotState.ZERO);
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.setSpeed(0);
    }
}
