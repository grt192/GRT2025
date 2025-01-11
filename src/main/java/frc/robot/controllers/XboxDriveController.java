package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * A single Xbox controller on port 0.
 */
public class XboxDriveController extends BaseDriveController {
    private final XboxController driveController = new XboxController(0);
     
    private final JoystickButton aButton = new JoystickButton(driveController, XboxController.Button.kA.value);
    private final JoystickButton bButton = new JoystickButton(driveController, XboxController.Button.kB.value);
    private final JoystickButton xButton = new JoystickButton(driveController, XboxController.Button.kX.value);
    private final JoystickButton yButton = new JoystickButton(driveController, XboxController.Button.kY.value);
    private final JoystickButton lBumper = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton rBumper = new JoystickButton(
        driveController, 
        XboxController.Button.kRightBumper.value
    );
    private final JoystickButton driveLStickButton = new JoystickButton(
        driveController, XboxController.Button.kLeftStick.value
    );
    private final JoystickButton driveRStickButton = new JoystickButton(
        driveController, XboxController.Button.kRightStick.value
    );

    @Override
    public double getForwardPower() {
        return -driveController.getLeftY();
    }

    @Override
    public double getLeftPower() {
        return -driveController.getLeftX();
    }

    @Override
    public double getRotatePower() {
        return -driveController.getRightX();
    }

    @Override
    public boolean getDriverHeadingResetButton() {
        return aButton.getAsBoolean();
    }

    @Override
    public boolean getLeftBumper() {
        return lBumper.getAsBoolean();
    }

    @Override
    public boolean getRightBumper() {
        return rBumper.getAsBoolean();
    }

    @Override
    public boolean getRelativeMode() {
        return driveController.getRightTriggerAxis() > .1;
    }

    @Override
    public void bindDriverHeadingReset(
        Runnable command, Subsystem requiredSubsystem
    ) {
        aButton.onTrue(new InstantCommand(
            command,
            requiredSubsystem
        ));
    }
}
