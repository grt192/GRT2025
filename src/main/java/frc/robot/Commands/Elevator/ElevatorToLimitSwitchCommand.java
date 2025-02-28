package frc.robot.Commands.Elevator;

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
        elevatorSubsystem.setPositionReference(ElevatorConstants.GROUND_POS);
    }

    @Override
    public boolean isFinished(){
        return elevatorSubsystem.getLimitSwitch();
    }

}
