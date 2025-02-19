package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

public class ElevatorToGroundCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorToGroundCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public void execute() {
        elevatorSubsystem.setTargetState(ElevatorState.GROUND);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atGround();
    }
}
