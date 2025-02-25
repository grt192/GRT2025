package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ElevatorToL3Command extends Command {
    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorToL3Command(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setPositionReference(ElevatorConstants.L3_POS);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.getClosedLoopError()
            < ElevatorConstants.ELEVATOR_TOLERANCE;
    }
}
