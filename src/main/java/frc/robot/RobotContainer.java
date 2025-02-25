// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controllers.PS5DriveController;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake.Pivot.PivotSubsystem;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Commands.Intake.Pivot.PivotToOuttakeCommand;
import frc.robot.Commands.Intake.Pivot.PivotToSourceCommand;
import frc.robot.Commands.Intake.Pivot.PivotUp90Command;
import frc.robot.Commands.Intake.Roller.RollerInCommand;
import frc.robot.Commands.Intake.Roller.RollerOutCommand;
import frc.robot.Commands.Intake.Roller.RollerStopCommand;
import frc.robot.Constants.VisionConstants;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private PS5DriveController driveController;
  private CommandPS5Controller mechController;
  
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private final RollerSubsystem rollerSubsystem = new RollerSubsystem();

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem visionSubsystem2 = new VisionSubsystem(
    VisionConstants.cameraConfigs[1]
  );
  private final VisionSubsystem visionSubsystem3 = new VisionSubsystem(
    VisionConstants.cameraConfigs[2]
  );
  private final VisionSubsystem visionSubsystem4 = new VisionSubsystem(
    VisionConstants.cameraConfigs[3]
  );

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    constructDriveController(); 
    constructMechController();
    bindIntake();
    // startLog();
    setVisionDataInterface();
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named f`actories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
      /* Driving -- One joystick controls translation, the other rotation. If the robot-relative button is held down,
      * the robot is controlled along its own axes, otherwise controls apply to the field axes by default. If the
      * swerve aim button is held down, the robot will rotate automatically to always face a target, and only
      * translation will be manually controllable. */
    swerveSubsystem.setDefaultCommand(
      new RunCommand(() -> {
        swerveSubsystem.setDrivePowers(
          driveController.getForwardPower(),
          driveController.getLeftPower(),
          driveController.getRotatePower()
        );
        }, 
        swerveSubsystem
      )
    );

    /* Pressing the button resets the field axes to the current robot axes. */
    driveController.bindDriverHeadingReset(
      () ->{
        swerveSubsystem.resetDriverHeading();
      },
      swerveSubsystem
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Three Meters");
  }

  /**
   * Constructs the drive controller based on the name of the controller at port
   * 0
   */
  private void constructDriveController(){
    DriverStation.refreshData();
    // if(DriverStation.getJoystickName(0).equals("Controller (Xbox One For Windows)")) {
    //     driveController = new XboxDriveController();
    // }
    // else if(DriverStation.getJoystickName(0).equals("DualSense Wireless Controller")){
        driveController = new PS5DriveController();
    // }
    // else{
    //     driveController = new DualJoystickDriveController();
    // }
    driveController.setDeadZone(0.05);
  }

  /**
   * Constructs the mech controller at port 1
   */
  private void constructMechController(){
    mechController = new CommandPS5Controller(1);
  }

  /**
   * Binds the intake commands to the mech controller
   */
  private void bindIntake(){
    pivotSubsystem.setDefaultCommand(
      new RunCommand(() -> {
        pivotSubsystem.setVelocityReference(mechController.getLeftY());
      },
      pivotSubsystem
      )
    );
    mechController.L2().onTrue(new RollerOutCommand(rollerSubsystem));
    mechController.L2().toggleOnFalse(new RollerStopCommand(rollerSubsystem));
    mechController.R2().onTrue(new RollerInCommand(rollerSubsystem));
    mechController.R2().toggleOnFalse(new RollerStopCommand(rollerSubsystem));
    mechController.povDown().onTrue(new PivotToOuttakeCommand(pivotSubsystem));
    mechController.povUp().onTrue(new PivotToSourceCommand(pivotSubsystem));
    mechController.L1().onTrue(new PivotUp90Command(pivotSubsystem));
  }

  /**
   * Starts datalog at /u/logs
   */
  private void startLog(){
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  /**
   * Links vision and swerve
   */
  private void setVisionDataInterface(){
    visionSubsystem2.setInterface(swerveSubsystem::addVisionMeasurements);
    visionSubsystem3.setInterface(swerveSubsystem::addVisionMeasurements);
    visionSubsystem4.setInterface(swerveSubsystem::addVisionMeasurements);

  }
}
