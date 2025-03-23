package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * A single Xbox controller on port 0.
 */
public class XboxDriveController extends BaseDriveController {

    private double deadZone = 0.02; 

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
        double forwardPower =  -driveController.getLeftY();
        if (Math.abs(forwardPower)> deadZone)
            return -driveController.getLeftY();
        else 
            return 0; 
    }

    @Override
    public double getLeftPower() {
        double leftPower =  -driveController.getLeftX();
        if (Math.abs(leftPower)> deadZone)
            return -driveController.getLeftX();
        else 
            return 0; 
    }

    @Override
    public double getRotatePower() {
        double rotatePower =  -driveController.getRightX();
        if (Math.abs(rotatePower)> deadZone)
            return -driveController.getRightX();
        else 
            return 0; 
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
    public Trigger getRelativeMode() {
        return null;
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

    @Override
    public void setDeadZone(double deadZone){

    }

    @Override
    public Trigger getAlignToReef() {
        return (Trigger) xButton;
    }

    @Override
    public Trigger getAlignToSource() {
        return (Trigger) aButton;
    }
}
