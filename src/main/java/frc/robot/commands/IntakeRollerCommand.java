package frc.robot.commands;

import frc.robot.subsystems.Intake.IntakeRollerSubsystem;

public class IntakeRollerCommand {

    private IntakeRollerSubsystem intakeRollerSubsystem;
    private double speed;

    public IntakeRollerCommand(IntakeRollerSubsystem intakeRollerSubsystem) {
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        this.speed = 0.5;

    }

    public void execute() {
        if (intakeRollerSubsystem.getSpeed() == 0) {
            intakeRollerSubsystem.setSpeed(speed);
        } else {
            intakeRollerSubsystem.stop();
        }

    }

    public void end() {
        intakeRollerSubsystem.stop();
    }

    public boolean isFinished() {
        return false;
    }

}
