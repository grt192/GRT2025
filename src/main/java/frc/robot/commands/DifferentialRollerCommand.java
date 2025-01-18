package frc.robot.commands;

import frc.robot.subsystems.Differential.DifferentialRollerSubsystem;

public class DifferentialRollerCommand {

    private DifferentialRollerSubsystem rollerSubsystem;
    private double speed;

    public DifferentialRollerCommand(DifferentialRollerSubsystem rollerSubsystem) {
        this.rollerSubsystem = rollerSubsystem;
        this.speed = 0.3;
    }

    public void execute() {
        if (rollerSubsystem.getSpeed() == 0) {
            rollerSubsystem.setSpeed(speed);
        } else {
            rollerSubsystem.stop();
        }
    }

    public void end() {
        rollerSubsystem.stop();
    }

    public boolean isFinished() {
        return false;
    }

}
