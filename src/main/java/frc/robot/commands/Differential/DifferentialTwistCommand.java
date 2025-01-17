package frc.robot.commands.Differential;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DiffySubsystem.DiffyArmSubsystem;

public class DifferentialTwistCommand extends Command {

    DiffyArmSubsystem leftDiffy;
    DiffyArmSubsystem rightDiffy;

    public DifferentialTwistCommand( DiffyArmSubsystem leftDiffy, DiffyArmSubsystem  rightDiffy) {
        
        this.leftDiffy = leftDiffy;
        this.rightDiffy = rightDiffy;

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
