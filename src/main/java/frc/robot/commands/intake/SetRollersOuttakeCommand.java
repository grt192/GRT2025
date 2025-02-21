package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.rollers.RollerSubsystem;

public class SetRollersOuttakeCommand extends Command{
    private final RollerSubsystem rollerSubsystem;
    private Timer rollTime;

    public SetRollersOuttakeCommand(RollerSubsystem rollerSubsystem) {
        this.rollerSubsystem = rollerSubsystem;
        rollTime = new Timer();
        rollTime.reset();
        addRequirements(rollerSubsystem);
    }
    @Override
    public void initialize() {
        rollTime.start();
    }

    @Override
    public void execute() {
        rollerSubsystem.setRollerPower(-.25);
    }

    @Override
    public boolean isFinished() {
        return rollTime.hasElapsed(1.5);
    }

    @Override
    public void end(boolean interrupted) {
        rollerSubsystem.setRollerPower(0);
    }
}
