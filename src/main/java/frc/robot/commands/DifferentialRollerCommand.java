package frc.robot.commands;

import frc.robot.subsystems.Differential.DifferentialRollerSubsystem;

/**
 * DifferentialRollerCommand controls the roller subsystem.
 */
public class DifferentialRollerCommand {

    private final DifferentialRollerSubsystem rollerSubsystem;
    private final double speed;

    /**
     * Constructor for DifferentialRollerCommand.
     * 
     * @param rollerSubsystem The roller subsystem to control.
     */
    public DifferentialRollerCommand(DifferentialRollerSubsystem rollerSubsystem) {
        this.rollerSubsystem = rollerSubsystem;
        this.speed = 0.3;
    }
  

    /**
     * Executes the command to control the roller subsystem.
     */
    public void execute() {
        if (rollerSubsystem.getSpeed() == 0) {
            rollerSubsystem.setSpeed(speed);
        } else {
            rollerSubsystem.stop();
        }
    }

    /**
     * Ends the command and stops the roller subsystem.
     */
    public void end() {
        rollerSubsystem.stop();
    }

    /**
     * Indicates whether the command is finished.
     * 
     * @return Always returns false as this command never finishes on its own.
     */
    public boolean isFinished() {
        return false;
    }
}
