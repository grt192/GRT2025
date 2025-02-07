package frc.robot.commands;

import frc.robot.subsystems.Intake.IntakeRollerSubsystem;

public class IntakeRollerCommand {

    private IntakeRollerSubsystem intakeRollerSubsystem;
    private double speed;

    public IntakeRollerCommand(IntakeRollerSubsystem intakeRollerSubsystem) {

    }

    public void execute() {

    }

    public void end() {
        intakeRollerSubsystem.stop();
    }

    public boolean isFinished() {
        return false;
    }

}
