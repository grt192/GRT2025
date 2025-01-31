package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ElevatorToL3Command extends Command {
    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorToL3Command(ElevatorSubsystem elevatorSubsystem) {
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void execute() {
        this.elevatorSubsystem.setTargetState(ElevatorState.L3);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atState(ElevatorState.L3);
    }
}