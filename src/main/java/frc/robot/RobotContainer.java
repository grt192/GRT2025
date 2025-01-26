// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.AutoAlignCommand;
import frc.robot.Commands.elevator.ElevatorToGroundCommand;
import frc.robot.Commands.elevator.ElevatorToL1Command;
import frc.robot.controllers.BaseDriveController;
import frc.robot.controllers.DualJoystickDriveController;
import frc.robot.controllers.PS5DriveController;
import frc.robot.controllers.XboxDriveController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.Elevator.ElevatorSubsystemTest;
import frc.robot.subsystems.FieldManagementSubsystem.FieldManagementSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Constants.VisionConstants;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private BaseDriveController driveController;

  private final CommandPS5Controller mechController = new CommandPS5Controller(1);

  // private final ElevatorSubsystemTest elevatorSubsystemTest = new ElevatorSubsystemTest();
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem visionSubsystem1 = new VisionSubsystem(
    VisionConstants.cameraConfigs[0]
  );
  private final VisionSubsystem visionSubsystem2 = new VisionSubsystem(
    VisionConstants.cameraConfigs[1]
  );


  private final Trigger xButton, sButton;
  private final Trigger lBumper, rBumper;


  // // private final PhoenixLoggingSubsystem phoenixLoggingSubsystem =
  //   // new PhoenixLoggingSubsystem(fieldManagementSubsystem);


  // private final SendableChooser<Command> autoChooser;

  boolean isCompetition = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    mechController = new CommandPS5Controller(1);

    xButton = new Trigger(mechController.cross());
    squareButton = new Trigger(mechController.square());
    xButton = new Trigger(mechController.cross());
    sButton = new Trigger(mechController.square());

    lBumper = new Trigger(mechController.L1());
    rBumper = new Trigger(mechController.R1());


    constructDriveController(); 
    startLog();
    setVisionDataInterface();
    configureBindings();
    

    // autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
    // (stream) -> isCompetition
    //   ? stream.filter(auto -> auto.getName().startsWith("C;"))
    //   : stream
    // );
    // SmartDashboard.putData("Auto Chooser", autoChooser);
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
  
    // xButton.onTrue(new InstantCommand(() -> {
    //   elevatorSubsystemTest.stop();
    // }));

    sButton.onTrue(new ConditionalCommand(
      new ElevatorToL1Command(elevatorSubsystem).withTimeout(4), 
      new ElevatorToGroundCommand(elevatorSubsystem).withTimeout(4), 
      () -> elevatorSubsystem.atState(ElevatorState.GROUND)));

    // elevatorSubsystemTest.setDefaultCommand(new InstantCommand(
    //   () -> {
    //     elevatorSubsystemTest.move(mechController.getL2Axis() - mechController.getR2Axis());
    //   },
    //   elevatorSubsystemTest
    // ));


  
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

    AutoAlignCommand.reefTest(swerveSubsystem).onlyWhile(
      () -> driveController.getForwardPower() <= 0.05 && 
      driveController.getLeftPower() <= 0.05
    );

    visionSubsystem.setInterface(swerveSubsystem::addVisionMeasurements);
    // driveController.getAlignToReef().onTrue(
    //   AutoAlignCommand.reefTest(swerveSubsystem).onlyWhile(() -> driveController.getForwardPower() 
    //   <= 0.05 && driveController.getLeftPower() <= 0.05));

    // visionSubsystem.setInterface(swerveSubsystem::addVisionMeasurements);
    driveController.getAlignToSource().onTrue(
      AutoAlignCommand.sourceTest(swerveSubsystem).onlyWhile(() -> driveController.getForwardPower() 
      <= 0.05 && driveController.getLeftPower() <= 0.05));

    AutoAlignCommand.reefTest(swerveSubsystem).onlyWhile(
      () -> driveController.getForwardPower() <= 0.05 && 
      driveController.getLeftPower() <= 0.05
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
   * Starts datalog at /media/sda1/robotLogs
   */
  private void startLog(){
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }
  /**
   * Links vision and swerve
   */
  private void setVisionDataInterface(){
    visionSubsystem1.setInterface(swerveSubsystem::addVisionMeasurements);
    visionSubsystem2.setInterface(swerveSubsystem::addVisionMeasurements);
  }
}
