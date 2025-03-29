package frc.robot.Commands.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ElevatorToCoralAlgaeCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorToCoralAlgaeCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setPositionReference(ElevatorConstants.coralAlgae_POS);
    }

    // @Override
    // public void end(boolean interrupted) {
    //     elevatorSubsystem.setPositionReference(elevatorSubsystem.getPosition());
    // }

    @Override
    public boolean isFinished() {
        return Math.abs(elevatorSubsystem.getPosition() - ElevatorConstants.coralAlgae_POS)
            < 5;
    }
}

