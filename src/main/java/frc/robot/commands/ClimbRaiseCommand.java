package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/** Lowers climb fully when off the chain. Should run at the start of auton to ensure that both arms are at their lowest
 *  positions so that the robot may fit underneath the stage. */
public class ClimbRaiseCommand extends Command {
    private final ClimbSubsystem climbSubsystem;
    private final int speed = 0; //placeholder
    private final double endingMotorPosition = 0.0;
    
    /** Constructs a new {@link ClimbLowerCommand}. */
    public ClimbRaiseCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;
        this.addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        climbSubsystem.driveRollers(speed);
    }

    @Override
    public boolean isFinished() {
        return climbSubsystem.getMotorPosition() == endingMotorPosition;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}