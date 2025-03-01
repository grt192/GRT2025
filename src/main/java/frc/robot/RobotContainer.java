// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controllers.PS5DriveController;

import java.util.EnumSet;
import java.util.jar.Attributes.Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Commands.Climb.StartClimbCommand;
import frc.robot.Commands.Climb.StopClimbCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.FieldManagementSubsystem.FieldManagementSubsystem;
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
import frc.robot.Commands.Elevator.ElevatorToAlgaeCommand;
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


  private Trigger climbTrigger;

  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  private final SendableChooser<Command> autoChooser;
  private boolean isCompetition = true;

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

  // private final VisionSubsystem visionSubsystem1 = new VisionSubsystem(
  //   VisionConstants.cameraConfigs[0]
  // );
  // private final VisionSubsystem visionSubsystem2 = new VisionSubsystem(
  //   VisionConstants.cameraConfigs[1]
  // );
  
  //driver camera stuff:
  private UsbCamera driverCamera;
  private MjpegServer driverCameraServer;
  private UsbCamera driverCamera2;

  private NetworkTableInstance ntInstance;
  private NetworkTable testTable; 
  private NetworkTable FMStable;
  private NetworkTableEntry cameraSelectionEntry;

  private final VisionSubsystem visionSubsystem2 = new VisionSubsystem(
    VisionConstants.cameraConfigs[1]
  );

  private final VisionSubsystem visionSubsystem3 = new VisionSubsystem(
    VisionConstants.cameraConfigs[2]
  );
  private final VisionSubsystem visionSubsystem4 = new VisionSubsystem(
    VisionConstants.cameraConfigs[3]
  );

  private final FieldManagementSubsystem fmsSubsystem = new FieldManagementSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    constructDriveController(); 
    constructMechController();

    bindClimb();

    bindElevator();
    bindIntake();

    // startLog();
    setVisionDataInterface();
    configureBindings();
    constructDriverCameras();
    constructNetworkTableListeners();

    NamedCommands.registerCommand("ElevatorToGround", new ElevatorToGroundCommand(elevatorSubsystem));
    NamedCommands.registerCommand("ElevatorToAlgae", new ElevatorToAlgaeCommand(elevatorSubsystem));
    NamedCommands.registerCommand("PivotToHorizontal", new PivotToHorizontalCommand(pivotSubsystem));
    NamedCommands.registerCommand("ElevatorToL4", new ElevatorToL4Command(elevatorSubsystem));
    NamedCommands.registerCommand("RollerIntake", new RollerInTillSensorCommand(rollerSubsystem));
    NamedCommands.registerCommand("PivotToOuttake", new PivotToOuttakeCommand(pivotSubsystem));
    NamedCommands.registerCommand("RollerOuttake", new RollerOutCommand(rollerSubsystem));
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
      ? stream.filter(auto -> auto.getName().startsWith("Pine"))
      : stream
    );
    SmartDashboard.putData("AutoChooser", autoChooser);
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
    return autoChooser.getSelected();
  }

  /**
   * Constructs the drive controller at port 0
   */
  private void constructDriveController(){
    driveController = new PS5DriveController();
    driveController.setDeadZone(0.05);
  }

  /**
   * Constructs mech controller at port 1
   */
  private void constructMechController(){
    mechController = new CommandPS5Controller(1);
  }

  /**
   * Binds climb commands to mech controller
   */
  private void bindClimb(){
    climbTrigger = mechController.create().and(mechController.options());
    climbTrigger.onTrue(new StartClimbCommand(climbSubsystem));
    climbTrigger.onFalse(new StopClimbCommand(climbSubsystem));
  /*
   * Binds elevator commands to mech controller
   */
  private void bindElevator(){

    // elevatorSubsystem.setDefaultCommand(
    //   new InstantCommand(() -> {
    //     elevatorSubsystem.setPower(-mechController.getLeftY());
    //   }, elevatorSubsystem)
    // );

    manualElevatorTrigger = new Trigger(
      () -> Math.abs(mechController.getRightY()) >= ElevatorConstants.CONTROLLER_DEADZONE
    );

    manualElevatorTrigger.onTrue(
      new InstantCommand(
        () -> {
          elevatorSubsystem.setPower(-mechController.getRightY());
        },
        elevatorSubsystem
      ).handleInterrupt(() -> elevatorSubsystem.setPower(0))
    );
    manualElevatorTrigger.toggleOnFalse(
      new InstantCommand(
        () -> {
          elevatorSubsystem.setPower(0);
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
    
    // manualPivotTrigger = new Trigger(
    //   () -> mechController.getLeftY() >= PivotConstants.CONTROLLER_DEADZONE
    // );

    // manualPivotTrigger.onTrue(
    //   new InstantCommand(
    //     () -> {
    //       pivotSubsystem.setDutyCycle(mechController.getLeftY());
    //     },
    //     pivotSubsystem
    //   )
    // );

    mechController.povDown().onTrue(
      new PivotToOuttakeCommand(pivotSubsystem)
    );
    
    // mechController.povLeft().onTrue(
    //   new PivotToHorizontalCommand(pivotSubsystem)
    // );

    mechController.povUp().onTrue(
      new PivotToSourceCommand(pivotSubsystem)
    );

    mechController.L1().onTrue(
      new PivotUp90Command(pivotSubsystem)
    );

    mechController.povRight().onTrue(
      new PivotToHorizontalCommand(pivotSubsystem)
    );

    // mechController.povUp().onTrue(
    //   new ConditionalCommand(
    //     new ParallelCommandGroup(
    //       new ElevatorToGroundCommand(elevatorSubsystem),
    //       new PivotUp90Command(pivotSubsystem)
    //     ),
    //     new SequentialCommandGroup(
    //       new ParallelCommandGroup(
    //         new ElevatorToSourceCommand(elevatorSubsystem),
    //         new PivotToSourceCommand(pivotSubsystem)
    //       ),
    //       new RollerInTillSensorCommand(rollerSubsystem),
    //       new WaitCommand(1),
    //       new ParallelCommandGroup(
    //         new ElevatorToGroundCommand(elevatorSubsystem),
    //         new PivotUp90Command(pivotSubsystem)
    //       )
    //     ),
    //     rollerSubsystem::getIntakeSensor
    //   )
    // );
  }

  private void bindRollers(){
    // manualRollerInTrigger = new Trigger(
    //   () -> 
    //     mechController.getL2Axis()
    //       >= RollerConstants.ROLLER_CONTROLLER_DEADZONE
    // );
    // manualRollerOutTrigger = new Trigger(
    //   () ->
    //     mechController.getR2Axis()
    //       >= RollerConstants.ROLLER_CONTROLLER_DEADZONE
    // );

    // manualRollerInTrigger.onTrue(
    //   new RollerInCommand(rollerSubsystem)
    // );

    // manualRollerOutTrigger.onTrue(
    //   new RollerOutCommand(rollerSubsystem)
    // );

    rollerSubsystem.setDefaultCommand(new ConditionalCommand(
      new InstantCommand( () -> {
      //ps5 trigger's range is -1 to 1, with non-input position being -1. This maps the range -1 to 1 to 0 to 1.
      rollerSubsystem.setRollerSpeed(.25 * (mechController.getR2Axis() + 1.) / 2.); 
      }, rollerSubsystem), 
      new InstantCommand( () -> {
      rollerSubsystem.setRollerSpeed(.07 * (mechController.getR2Axis() - mechController.getL2Axis()));
      }, rollerSubsystem), 
      () -> rollerSubsystem.getIntakeSensor()));
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

    //visionSubsystem1.setInterface(swerveSubsystem::addVisionMeasurements);
    //visionSubsystem2.setInterface(swerveSubsystem::addVisionMeasurements);
    visionSubsystem3.setInterface(swerveSubsystem::addVisionMeasurements);
    visionSubsystem4.setInterface(swerveSubsystem::addVisionMeasurements);

    // visionSubsystem2.setInterface(swerveSubsystem::addVisionMeasurements);
    // visionSubsystem3.setInterface(swerveSubsystem::addVisionMeasurements);
    // visionSubsystem4.setInterface(swerveSubsystem::addVisionMeasurements);

  }

  public void constructDriverCameras(){
    try {
      driverCamera = new UsbCamera("fisheye", 0);
      driverCamera.setVideoMode(PixelFormat.kMJPEG, 160, 120, 30);
      driverCamera.setExposureManual(40);
      driverCameraServer = new MjpegServer("m1", 1181);
      driverCameraServer.setSource(driverCamera);
    } catch (Exception e) {
      System.out.print(e);
    }
    try {
      driverCamera2 = new UsbCamera("fisheye2", 1);
      driverCamera2.setVideoMode(PixelFormat.kMJPEG, 160, 120, 30);
      driverCamera2.setExposureManual(40);
    } catch (Exception e) {
      System.out.print(e);
    }
  }

  public void constructNetworkTableListeners(){

    ntInstance = NetworkTableInstance.getDefault();
    FMStable = ntInstance.getTable("FMSInfo");
    testTable = ntInstance.getTable("testTable");

    //allianceEntry = FMStable.getEntry("IsRedAlliance");
    cameraSelectionEntry = testTable.getEntry("cameraSelection");
    // FMStable.addListener("IsRedAlliance", EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) -> {
            
    //     });
    testTable.addListener("cameraSelection", EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) ->{
      if (event.valueData.value.getBoolean()){
        //server1.setSource(camera1);
        driverCameraServer.setSource(driverCamera);

        System.out.println("CAMERA1!");
      }
      else{
        //server1.setSource(camera2);
        driverCameraServer.setSource(driverCamera2);

        System.out.println("CAMERA2!");

      }
    });

  }



}
