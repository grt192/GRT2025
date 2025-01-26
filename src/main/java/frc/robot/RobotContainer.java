// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.commands.pivot.SetPivotVerticalCommand;
// import frc.robot.commands.pivot.SetPivotZeroCommand;
import frc.robot.controllers.BaseDriveController;
import frc.robot.controllers.DualJoystickDriveController;
import frc.robot.controllers.PS5DriveController;
import frc.robot.controllers.XboxDriveController;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.FieldManagementSubsystem.FieldManagementSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.intake.pivot.PivotState;
import frc.robot.subsystems.intake.pivot.PivotSubsystem;
import frc.robot.subsystems.intake.rollers.RollerSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.pivot.SetPivotOuttakeCommand;
import frc.robot.commands.pivot.SetPivotSourceCommand;
import frc.robot.commands.pivot.SetPivotVerticalCommand;
import frc.robot.commands.pivot.SetPivotZeroCommand;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private BaseDriveController driveController;

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private final RollerSubsystem rollerSubsystem = new RollerSubsystem();
  private final VisionSubsystem visionSubsystem2 = new VisionSubsystem(
    VisionConstants.cameraConfigs[1]
  );
  private final VisionSubsystem visionSubsystem3 = new VisionSubsystem(
    VisionConstants.cameraConfigs[2]
  );
  private final VisionSubsystem visionSubsystem4 = new VisionSubsystem(
    VisionConstants.cameraConfigs[3]
  );

  private CommandPS5Controller mechController;
  private Trigger aButton, lTrigger, rTrigger, lBumper, rBumper;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    mechController = new CommandPS5Controller(1);
    aButton = new Trigger(mechController.cross());
    lTrigger = new Trigger(mechController.L2());
    rTrigger = new Trigger(mechController.R2());

    lBumper = new Trigger(mechController.L1());
    rBumper = new Trigger(mechController.R1());

    constructDriveController(); 
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
    // swerveSubsystem.setDefaultCommand(
    //   new RunCommand(() -> {
    //     swerveSubsystem.setDrivePowers(
    //       driveController.getForwardPower(),
    //       driveController.getLeftPower(),
    //       driveController.getRotatePower()
    //     );
    //     }, 
    //     swerveSubsystem
    //   )
    // );

    rBumper.onTrue(
      new ConditionalCommand(
        new SetPivotOuttakeCommand(pivotSubsystem),
        new ConditionalCommand(
          new SetPivotZeroCommand(pivotSubsystem).andThen(new SetPivotSourceCommand(pivotSubsystem)), 
          new SetPivotSourceCommand(pivotSubsystem), 
          () -> (pivotSubsystem.getCurrentAngle() < PivotState.ZERO.getTargetAngle())),
        () -> (pivotSubsystem.getTargetState() == PivotState.SOURCE)
        )
    );

    lBumper.onTrue(
      new SetPivotVerticalCommand(pivotSubsystem).withTimeout(2.5)
    );

    rollerSubsystem.setDefaultCommand(new ConditionalCommand(
      new InstantCommand( () -> {
        //ps5 trigger's range is -1 to 1, with non-input position being -1. This maps the range -1 to 1 to 0 to 1.
        rollerSubsystem.setRollerPower(.25 * (mechController.getR2Axis() + 1.) / 2.); 
      }, rollerSubsystem), 
      new InstantCommand( () -> {
        rollerSubsystem.setRollerPower(.25 * (mechController.getR2Axis() - mechController.getL2Axis()));
      }, rollerSubsystem), 
      () -> rollerSubsystem.getIntakeSensor()));

    // aButton.onTrue(
    //     new SetPivotVerticalCommand(pivotSubsystem)
    // );

    // rollerSubsystem.setDefaultCommand(new InstantCommand( () -> {
    //   rollerSubsystem.setRollerPower(mechController.getL2Axis() - mechController.getR2Axis());
    // }, rollerSubsystem));

    // /* Pressing the button resets the field axes to the current robot axes. */
    // driveController.bindDriverHeadingReset(
    //   () ->{
    //     swerveSubsystem.resetDriverHeading();
    //   },
    //   swerveSubsystem
    // );
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
    if(DriverStation.getJoystickName(0).equals("Controller (Xbox One For Windows)")) {
        driveController = new XboxDriveController();
    }
    else if(DriverStation.getJoystickName(0).equals("DualSense Wireless Controller")){
        driveController = new PS5DriveController();
    }
    else{
        driveController = new DualJoystickDriveController();
    }
    driveController.setDeadZone(0.05);
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
