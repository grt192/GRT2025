package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

public class ElevatorToLimitSwitchCommand extends Command{
    private ElevatorSubsystem elevatorSubsystem;

    public ElevatorToLimitSwitchCommand(ElevatorSubsystem elevatorSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.setPower(-.3);
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setPower(0);
    }
}
