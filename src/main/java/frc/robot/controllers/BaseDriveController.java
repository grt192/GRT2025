package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** The base class for a drive controller. Contains all needed methods for driving the robot (without mechs) */
public abstract class BaseDriveController {

    /**
     * Gets the forward power commanded by the controller.
     *
     * @return The [-1.0, 1.0] forward power.
     */
    public abstract double getForwardPower();

    /**
     * Gets the left power commanded by the controller.
     *
     * @return The [-1.0, 1.0] left power.
     */
    public abstract double getLeftPower();

    /**
     * Gets the rotational power commanded by the controller.
     *
     * @return The [-1.0, 1.0] angular power.
     */
    public abstract double getRotatePower();

    /**
     * Gets the button to reset the driver heading.
     *
     * @return The JoystickButton to reset the driver heading. 
     */
    public abstract boolean getDriverHeadingResetButton();

    /**
     * Gets the left bumper or equivalent. Used in testSingleModuleSwerveSubsystem to move between tests.
     *
     * @return The JoystickButton of the left bumper or equivalent.
     */
    public abstract boolean getLeftBumper();

    /**
     * Gets the right bumper or equivalent. Used in testSingleModuleSwerveSubsystem to move between tests.
     *
     * @return The JoystickButton of the right bumper or equivalent.
     */
    public abstract boolean getRightBumper();

    /**
     * Gets whether the driver is currently running in relative mode or not. Relative mode means moving the joystick
     * forward will move the robot in the direction of the intake. The default driving mode is field relative, meaning
     * forward is a direction set on the field by the driver (see resetDriverHeading)
     *
     * @return true if relativeMode is active, false otherwise
     */
    public abstract Trigger getRelativeMode();

    // public abstract boolean getAlignToReef();

    public abstract void bindDriverHeadingReset(
        Runnable command, Subsystem requiredSubsystem
    );

    public abstract void setDeadZone(double deadZone);

    public abstract Trigger getAlignToReef();

    public abstract Trigger getAlignToSource();
    
}
