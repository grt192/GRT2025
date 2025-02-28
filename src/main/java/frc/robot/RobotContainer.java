// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controllers.PS5DriveController;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.Pivot.PivotSubsystem;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Commands.Intake.Pivot.PivotToHorizontalCommand;
import frc.robot.Commands.Intake.Pivot.PivotToOuttakeCommand;
import frc.robot.Commands.Intake.Pivot.PivotToSourceCommand;
import frc.robot.Commands.Intake.Pivot.PivotUp90Command;
import frc.robot.Commands.Intake.Pivot.PivotZeroTo90Command;
import frc.robot.Commands.Intake.Roller.RollerInCommand;
import frc.robot.Commands.Intake.Roller.RollerInTillSensorCommand;
import frc.robot.Commands.Intake.Roller.RollerOutCommand;
import frc.robot.Commands.Intake.Roller.RollerStopCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.IntakeConstans.PivotConstants;
import frc.robot.Constants.IntakeConstans.RollerConstants;
import frc.robot.Commands.Elevator.ElevatorToGroundCommand;
import frc.robot.Commands.Elevator.ElevatorToL1Command;
import frc.robot.Commands.Elevator.ElevatorToL2Command;
import frc.robot.Commands.Elevator.ElevatorToL3Command;
import frc.robot.Commands.Elevator.ElevatorToL4Command;
import frc.robot.Commands.Elevator.ElevatorToLimitSwitchCommand;
import frc.robot.Commands.Elevator.ElevatorToSourceCommand;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private PS5DriveController driveController;
  private CommandPS5Controller mechController;
  private Trigger manualElevatorTrigger;
  private Trigger manualPivotTrigger;
  private Trigger manualRollerInTrigger;
  private Trigger manualRollerOutTrigger;
  
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private final RollerSubsystem rollerSubsystem = new RollerSubsystem();

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
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
    bindElevator();
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
    driveController = new PS5DriveController();
    driveController.setDeadZone(0.05);
  }

  /**
   * Constructs mech controller
   */
  private void constructMechController(){
    mechController = new CommandPS5Controller(1);
  }

  /**
   * Binds elevator commands to mech controller
   */
  private void bindElevator(){

    // elevatorSubsystem.setDefaultCommand(
    //   new InstantCommand(() -> {
    //     elevatorSubsystem.setDutyCycle(-mechController.getLeftY());
    //   }, elevatorSubsystem)
    // );

    manualElevatorTrigger = new Trigger(
      () -> mechController.getRightY() >= ElevatorConstants.CONTROLLER_DEADZONE
    );

    manualElevatorTrigger.onTrue(
      new InstantCommand(
        () -> {
          elevatorSubsystem.setDutyCycle(mechController.getRightY());
        },
        elevatorSubsystem
      )
    );

    mechController.R1().onTrue(new ElevatorToLimitSwitchCommand(elevatorSubsystem));
    mechController.triangle().onTrue(new ElevatorToL4Command(elevatorSubsystem));
    mechController.circle().onTrue(new ElevatorToL3Command(elevatorSubsystem));
    mechController.square().onTrue(new ElevatorToL2Command(elevatorSubsystem));
    mechController.cross().onTrue(new ElevatorToSourceCommand(elevatorSubsystem));
  }

  private void bindPivot(){
    // pivotSubsystem.setDefaultCommand(
    //   new InstantCommand(() -> {
    //     pivotSubsystem.setDutyCycle(mechController.getRightY());
    //   },
    //   pivotSubsystem
    //   )
    // );
    
    manualPivotTrigger = new Trigger(
      () -> mechController.getLeftY() >= PivotConstants.CONTROLLER_DEADZONE
    );

    manualPivotTrigger.onTrue(
      new InstantCommand(
        () -> {
          pivotSubsystem.setDutyCycle(mechController.getLeftY());
        },
        pivotSubsystem
      )
    );

    mechController.povDown().onTrue(
      new PivotToOuttakeCommand(pivotSubsystem)
    );
    
    mechController.povLeft().onTrue(
      new PivotToHorizontalCommand(pivotSubsystem)
    );

    mechController.povRight().onTrue(
      new PivotToSourceCommand(pivotSubsystem)
    );

    mechController.povUp().onTrue(
      new ConditionalCommand(
        new ParallelCommandGroup(
          new ElevatorToGroundCommand(elevatorSubsystem),
          new PivotUp90Command(pivotSubsystem)
        ),
        new SequentialCommandGroup(
          new ParallelCommandGroup(
            new ElevatorToSourceCommand(elevatorSubsystem),
            new PivotToSourceCommand(pivotSubsystem)
          ),
          new RollerInTillSensorCommand(rollerSubsystem),
          new ParallelCommandGroup(
            new ElevatorToGroundCommand(elevatorSubsystem),
            new PivotUp90Command(pivotSubsystem)
          )
        ),
        rollerSubsystem::getIntakeSensor
      )
    );
  }

  private void bindRollers(){
    manualRollerInTrigger = new Trigger(
      () -> 
        mechController.getL2Axis()
          >= RollerConstants.ROLLER_CONTROLLER_DEADZONE
    );
    manualRollerOutTrigger = new Trigger(
      () ->
        mechController.getR2Axis()
          >= RollerConstants.ROLLER_CONTROLLER_DEADZONE
    );

    manualRollerInTrigger.onTrue(
      new RollerInCommand(rollerSubsystem)
    );

    manualRollerOutTrigger.onTrue(
      new RollerOutCommand(rollerSubsystem)
    );
  }
  //Binds the intake commands to the mech controller
  private void bindIntake(){
    // mechController.R1().onTrue(new PivotZeroTo90Command(pivotSubsystem));
    bindPivot();
    bindRollers();
    // rollerSubsystem.setDefaultCommand(new ConditionalCommand(
    //   new InstantCommand( () -> {
    //     //ps5 trigger's range is -1 to 1, with non-input position being -1. This maps the range -1 to 1 to 0 to 1.
    //     rollerSubsystem.setRollerSpeed(.25 * (me   chController.getL2Axis() + 1.) / 2.); 
    //   }, rollerSubsystem), 
    //   new InstantCommand( () -> {
    //     rollerSubsystem.setRollerSpeed(.15 * (mechController.getL2Axis() - mechController.getR2Axis()));
    //   }, rollerSubsystem), 
    //   () -> rollerSubsystem.getIntakeSensor()));
      
    // mechController.povUp().onTrue(
    //   new ConditionalCommand(
    //     new PivotToSourceCommand(pivotSubsystem),
    //     new PivotToHorizontalCommand(pivotSubsystem).andThen(new PivotToSourceCommand(pivotSubsystem)),
    //     () -> pivotSubsystem.getPosition() > 0
    //     ));
    
    // mechController.L1().onTrue(
    //   new ConditionalCommand(
    //     new PivotUp90Command(pivotSubsystem),
    //     new PivotToHorizontalCommand(pivotSubsystem).andThen(new PivotUp90Command(pivotSubsystem)),
    //     () -> pivotSubsystem.getPosition() > 0
    //     ));

    // mechController.L1().onTrue(
    //   new PivotUp90Command(pivotSubsystem)
    // );
    // mechController.povDown().onTrue(
    //   new PivotToOuttakeCommand(pivotSubsystem)
    // );

    // mechController.povRight().onTrue(
    //   new PivotToSourceCommand(pivotSubsystem)
    // );

    // mechController.povUp().onTrue(
    //   new PivotToHorizontalCommand(pivotSubsystem)
    // );
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
    // visionSubsystem2.setInterface(swerveSubsystem::addVisionMeasurements);
    // visionSubsystem3.setInterface(swerveSubsystem::addVisionMeasurements);
    // visionSubsystem4.setInterface(swerveSubsystem::addVisionMeasurements);

  }
}
