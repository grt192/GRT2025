package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorToL4Command extends Command {
    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorToL4Command(ElevatorSubsystem elevatorSubsystem) {
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void execute() {
        elevatorSubsystem.setTargetState(ElevatorState.L4);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atState(ElevatorState.L4);
    }
}