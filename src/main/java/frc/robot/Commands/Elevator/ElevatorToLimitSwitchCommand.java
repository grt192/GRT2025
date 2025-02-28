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
        elevatorSubsystem.setDutyCycle(
            ElevatorConstants.DUTY_CYCLE_TO_GROUND_SPEED
        );
    }

    @Override
    public boolean isFinished(){
        return elevatorSubsystem.getLimitSwitch();
    }

    @Override
    public void end(boolean interrupted) {
        elevatorSubsystem.setDutyCycle(0);
    }
}
