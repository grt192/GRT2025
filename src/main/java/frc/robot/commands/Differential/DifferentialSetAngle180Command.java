package frc.robot.commands.Differential;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DiffyConstants;
import frc.robot.subsystems.DiffySubsystem.DiffyArmSubsystem;
import frc.robot.subsystems.DiffySubsystem.DiffyState;;

public class DifferentialSetAngle180Command extends Command{

    DiffyArmSubsystem diffyArmSubsystem;
    DiffyState diffyState;

    public void DifferentialSetAngle90Command (DiffyArmSubsystem diffyArmSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        // Configure additional PID settings by calling getPIDController() and setting the desired values

        this.diffyArmSubsystem = diffyArmSubsystem;
        this.diffyState = DiffyState.ANGLE_180;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() { //figure out order
        diffyArmSubsystem.setArmPosition(diffyState.getDiffyArmPos());
        diffyArmSubsystem.setWristPosition(diffyState.getDiffyWristPos());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (diffyArmSubsystem.getDiffyArmPosition() - diffyState.getDiffyArmPos() < DiffyConstants.TOLERANCE) && 
        (diffyArmSubsystem.getDiffyArmPosition() - diffyState.getDiffyWristPos() < DiffyConstants.TOLERANCE);
    }

}
