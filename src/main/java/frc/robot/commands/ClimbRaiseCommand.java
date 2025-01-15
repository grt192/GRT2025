package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/** Lowers climb fully when off the chain. Should run at the start of auton to ensure that both arms are at their lowest
 *  positions so that the robot may fit underneath the stage. */
public class ClimbRaiseCommand extends Command {
    private static final double LOWERING_SPEED = -0.9;
    private final ClimbSubsystem climbSubsystem;
    private final int speed = 0; //placeholder
    
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
        return climbSubsystem.isLowered();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}