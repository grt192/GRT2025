package frc.robot.Commands.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ElevatorToAlgaeCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorToAlgaeCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setPositionReference(ElevatorConstants.Algae_POS);
    }

    // @Override
    // public void end(boolean interrupted) {
    //     elevatorSubsystem.setPositionReference(elevatorSubsystem.getPosition());
    // }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getClosedLoopError()
            < ElevatorConstants.ELEVATOR_TOLERANCE;
    }
}


