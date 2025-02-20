package frc.robot.commands.pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.pivot.PivotState;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;

public class SetPivotVerticalCommand extends Command{
        private PivotSubsystem pivotSubsystem;

    public SetPivotVerticalCommand(PivotSubsystem pivotSubsystem) {
        this.addRequirements(pivotSubsystem);
        this.pivotSubsystem = pivotSubsystem;
    }

    @Override
    public void execute() {
        this.pivotSubsystem.setState(PivotState.VERTICAL);
        // System.out.println("vertical" + pivotSubsystem.getPositionError());
    }

    @Override
    public boolean isFinished() {
        return pivotSubsystem.atState(PivotState.VERTICAL);
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.setSpeed(0);
    }
}
