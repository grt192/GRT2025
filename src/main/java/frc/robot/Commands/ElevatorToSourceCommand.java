package frc.robot.Commands;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;

/** Sends the elevator to its amp-scoring position. */
public class ElevatorToSourceCommand extends Command {
    private ElevatorSubsystem elevatorSubsystem;
    // private NetworkTableInstance nt;
    // private NetworkTable table;
    // private NetworkTableEntry entry;

    /** Constructs a {@link ElevatorToAmpCommand} for the specified elevator. */
    public ElevatorToSourceCommand(ElevatorSubsystem elevatorSubsystem) {
        this.addRequirements(elevatorSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
    }

    @Override
    public void end(boolean interrupted) {

        // nt = NetworkTableInstance.getDefault();
        // table = nt.getTable("elevator");
        // entry = table.getEntry("position");
        // entry.setInteger(3);

    }

    @Override
    public void execute() {
        this.elevatorSubsystem.setTargetState(ElevatorState.SOURCE);
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.atState(ElevatorState.SOURCE);
    }
}