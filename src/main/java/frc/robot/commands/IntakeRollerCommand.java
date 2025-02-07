package frc.robot.commands;

import frc.robot.subsystems.Intake.IntakeRollerSubsystem;
import frc.robot.Constants;

public class IntakeRollerCommand {

    private IntakeRollerSubsystem intakeRollerSubsystem;
    private double rollerSpeed;


    public IntakeRollerCommand(IntakeRollerSubsystem intakeRollerSubsystem) {
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        this.rollerSpeed = Constants.DifferentialRollerConstants.ROLLER_SPEED;
    

    }

    public void execute() {
        if (intakeRollerSubsystem.getSpeed() == 0) {
            intakeRollerSubsystem.setSpeed(rollerSpeed);
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
