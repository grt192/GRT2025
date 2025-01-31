package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ElevatorToL2Command extends Command {
    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorToL2Command(ElevatorSubsystem elevatorSubsystem) {
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void execute() {
        this.elevatorSubsystem.setTargetState(ElevatorState.L2);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atState(ElevatorState.L2);
    }
}