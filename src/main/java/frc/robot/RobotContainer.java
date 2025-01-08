// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.SingleModuleSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  int drivePort = 1;
  int steerPort = 2;
  int offsetRads = 0;

  public double count;

            
  int controllerSwitch = 0;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController mechController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  SwerveModule mod = new SwerveModule(drivePort, steerPort, offsetRads);
  // IAmDyingSubsystem pls = new IAmDyingSubsystem();
  SingleModuleSwerveSubsystem singleModuleSwerve = new SingleModuleSwerveSubsystem(mod);

  int state = 0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    double count = 0;
    // pls.configurePID(.5, 0, 0, 0); 
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    singleModuleSwerve.setDefaultCommand(new InstantCommand(() -> {
      // System.out.println(mod.getWrappedAngle());
      if(mechController.getRightBumperPressed()) {
        if (state == 7) {
          state = 0;
        }
        else {
          state += 1;
        }
      }

      switch (state) {
          case 1: 
              singleModuleSwerve.setRawPowers(.5, 0);
              // singleModuleSwerve.setState(0, Units.degreesToRadians(180));
              break;

          case 2:
              singleModuleSwerve.setRawPowers(-.5, 0);
              // singleModuleSwerve.setState(0, Units.degreesToRadians(180));
              break;

          case 3:
              singleModuleSwerve.setRawPowers(0, .5);
              break;

          case 4:
              singleModuleSwerve.setRawPowers(0, -.5);
              break;

          case 5:
              singleModuleSwerve.setState(0, Units.degreesToRadians(45));
              break;

          case 6:
              singleModuleSwerve.setState(0, Units.degreesToRadians(90));
              break;
          
          case 7:
              singleModuleSwerve.setState(0, Units.degreesToRadians(135));
              break;
              
          case 0:
              singleModuleSwerve.setRawPowers(0, 0);
              break;

          default:
              break;
      }
    }, singleModuleSwerve));

  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
