package frc.robot.commands.Differential;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DiffySubsystem.DiffyArmSubsystem;

public class DiffySetPivotCommand extends Command{

    DiffyArmSubsystem diffyArmSubsystem;
    double setAngle;

    public DiffySetPivotCommand( DiffyArmSubsystem  diffyArmSubsystem, double setAngle)  {
        this.diffyArmSubsystem = diffyArmSubsystem;
        this.setAngle = setAngle;
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID settings by calling getPIDController() and setting the desired values
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println(diffyArmSubsystem.getDiffyArmPosition());
        diffyArmSubsystem.setArmPosition(Units.degreesToRadians(setAngle));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;

    }

 }

