package frc.robot.commands.Differential;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DiffySubsystem.DiffyArmSubsystem;
import frc.robot.subsystems.DiffySubsystem.DiffyState;

public class DiffyTestWrist90Command extends Command{

    DiffyArmSubsystem diffyArmSubsystem;

    public DiffyTestWrist90Command(DiffyArmSubsystem diffyArmSubsystem)  {
        this.diffyArmSubsystem = diffyArmSubsystem;
        addRequirements(diffyArmSubsystem);
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID settings by calling getPIDController() and setting the desired values
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        diffyArmSubsystem.setWristPosition(DiffyState.WTEST_90.getDiffyWristPos());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return diffyArmSubsystem.atWristState(DiffyState.WTEST_90);
    }

 }

