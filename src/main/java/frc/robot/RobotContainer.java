// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// frc imports
import frc.robot.controllers.PS5DriveController;

// Subsystems
import frc.robot.subsystems.Elevator.ElevatorSubsystem;
import frc.robot.subsystems.FieldManagementSubsystem.FieldManagementSubsystem;
import frc.robot.subsystems.Intake.Pivot.PivotSubsystem;
import frc.robot.subsystems.Intake.Roller.RollerSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.Climb.ClimbSubsystem;

// Commands - Intake Pivot
import frc.robot.Commands.Intake.Pivot.PivotToHorizontalCommand;
import frc.robot.Commands.Intake.Pivot.PivotToL4Command;
import frc.robot.Commands.Intake.Pivot.PivotToOuttakeCommand;
import frc.robot.Commands.Intake.Pivot.PivotToSourceCommand;
import frc.robot.Commands.Intake.Pivot.PivotUp90Command;
import frc.robot.Commands.Intake.Pivot.PivotToBarge;
import frc.robot.Commands.Intake.Pivot.PivotToGroundAlgaeCommand;
import frc.robot.Commands.Intake.Pivot.PivotZeroTo90Command;

// Commands - Intake Roller
import frc.robot.Commands.Intake.Roller.RollerInCommand;
import frc.robot.Commands.Intake.Roller.RollerInTillSensorCommand;
import frc.robot.Commands.Intake.Roller.RollerOutCommand;
import frc.robot.Commands.Intake.Roller.RollerStopCommand;

// Commands - Elevator
import frc.robot.Commands.Elevator.ElevatorToAlgaeCommand;
import frc.robot.Commands.Elevator.ElevatorToGroundAlgaeCommand;
import frc.robot.Commands.Elevator.ElevatorToGroundCommand;
import frc.robot.Commands.Elevator.ElevatorToL1Command;
import frc.robot.Commands.Elevator.ElevatorToL2Command;
import frc.robot.Commands.Elevator.ElevatorToL3Command;
import frc.robot.Commands.Elevator.ElevatorToL4Command;
import frc.robot.Commands.Elevator.ElevatorToLimitSwitchCommand;
import frc.robot.Commands.Elevator.ElevatorToSourceCommand;
import frc.robot.Commands.Align.LRReefAlignCommand;
// Commands - Climb
import frc.robot.Commands.Climb.StartClimbCommand;
import frc.robot.Commands.Climb.StopClimbCommand;

// Constants
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.IntakeConstans.PivotConstants;
import frc.robot.Constants.IntakeConstans.RollerConstants;

// PathPlanner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

// WPILib imports
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Intake.Pivot.PivotToL4Command;

// Java Standard Library
import java.util.EnumSet;
import java.util.jar.Attributes.Name;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  private boolean isCompetition = true;

  private PS5DriveController driveController;
  private CommandPS5Controller mechController;
  private Trigger manualElevatorTrigger;
  private Trigger manualPivotTrigger;
  private Trigger aButton;
  private Trigger driveLBumper, driveRBumper;

  private Trigger createTrigger, optionTrigger;
  private Boolean hasPiece = false;
  private Boolean yankAlgae = false;
  
  private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
  private final RollerSubsystem rollerSubsystem = new RollerSubsystem();

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();



  //driver camera stuff:
  private UsbCamera driverCamera;
  private MjpegServer driverCameraServer;

  private NetworkTableInstance ntInstance;
  private NetworkTable testTable; 
  private NetworkTable FMStable;
  private NetworkTableEntry cameraSelectionEntry;
  private NetworkTableEntry autonTableEntry;
  private NetworkTableEntry autonTableSelection;
  private final VisionSubsystem visionSubsystem1 = new VisionSubsystem(
    VisionConstants.cameraConfigs[0]
  );
  
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

    createTrigger = new Trigger(mechController.create());
    optionTrigger = new Trigger(mechController.options());
    aButton = new Trigger(mechController.cross());

    driveLBumper = new Trigger(() -> driveController.getLeftBumper());
    driveRBumper = new Trigger(() -> driveController.getRightBumper());
    

    bindElevator();
    bindIntake();
    bindClimb();

    // startLog();
    setVisionDataInterface();
    configureBindings();
    constructDriverCameras();
    constructNetworkTableListeners();

    NamedCommands.registerCommand("ElevatorToGround", new ElevatorToGroundCommand(elevatorSubsystem));
    NamedCommands.registerCommand("ElevatorToAlgae", new ElevatorToAlgaeCommand(elevatorSubsystem));
    NamedCommands.registerCommand("PivotToHorizontal", new PivotToHorizontalCommand(pivotSubsystem));
    NamedCommands.registerCommand("ElevatorToL4", new ElevatorToL4Command(elevatorSubsystem));
    NamedCommands.registerCommand("RollerIntake", new RollerInCommand(rollerSubsystem));
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

    
    driveLBumper.onTrue(
      new LRReefAlignCommand(swerveSubsystem, fmsSubsystem, false).onlyWhile(() -> driveController.getForwardPower() 
      <= 0.05 && driveController.getLeftPower() <= 0.05));
    
    driveRBumper.onTrue(
      new LRReefAlignCommand(swerveSubsystem, fmsSubsystem, true).onlyWhile(() -> driveController.getForwardPower() 
      <= 0.05 && driveController.getLeftPower() <= 0.05));
    

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
   * Binds climb commands to mech controller
   */
  private void bindClimb(){
    
    createTrigger.and(optionTrigger).whileTrue(
      new RunCommand(() -> {
        climbSubsystem.setTorqueCurrentFOC(60);

      }, climbSubsystem)
    ).onFalse(
      new RunCommand(() -> {
        climbSubsystem.setPower(0);
      }, climbSubsystem)
    );
  }

  /**
   * Binds elevator commands to mech controller
   */
  private void bindElevator(){

    manualElevatorTrigger = new Trigger(
      () -> Math.abs(mechController.getRightY()) >= ElevatorConstants.CONTROLLER_DEADZONE
    );

    manualElevatorTrigger.onTrue(
      new RunCommand(
        () -> {
          elevatorSubsystem.setPower(-mechController.getRightY());
        },
        elevatorSubsystem
      ).handleInterrupt(() -> elevatorSubsystem.setPower(0))
    );
    manualElevatorTrigger.onFalse( //toggleonFalse
      new RunCommand(
        () -> {
          elevatorSubsystem.setPower(0);
        },
        elevatorSubsystem
      )
    );

    mechController.povDown().onTrue(new ElevatorToLimitSwitchCommand(elevatorSubsystem).alongWith(new PivotUp90Command(pivotSubsystem)));
    mechController.povRight().onTrue(new ElevatorToAlgaeCommand(elevatorSubsystem).alongWith(new PivotToL4Command(pivotSubsystem)));

    mechController.povUp().onTrue((new PivotToBarge(pivotSubsystem)).andThen(new ElevatorToL4Command(elevatorSubsystem)));
    mechController.povLeft().onTrue(new ElevatorToSourceCommand(elevatorSubsystem).alongWith(new PivotToSourceCommand(pivotSubsystem)));

    //UPDATED
    mechController.triangle().onTrue(new ElevatorToL4Command(elevatorSubsystem).andThen(new PivotToL4Command(pivotSubsystem)));
    mechController.circle().onTrue(new ElevatorToL3Command(elevatorSubsystem).alongWith(new PivotToOuttakeCommand(pivotSubsystem)));
    mechController.square().onTrue(new ElevatorToL2Command(elevatorSubsystem).alongWith(new PivotToOuttakeCommand(pivotSubsystem)));
    mechController.cross().onTrue(new ElevatorToSourceCommand(elevatorSubsystem).alongWith(new PivotToSourceCommand(pivotSubsystem)));
    mechController.L1().onTrue(new ElevatorToGroundAlgaeCommand(elevatorSubsystem).alongWith(new PivotToGroundAlgaeCommand(pivotSubsystem)));
    //
  }

  private void bindPivotOverride(){
    
    manualPivotTrigger = new Trigger(
      () -> Math.abs(mechController.getLeftY()) >= PivotConstants.CONTROLLER_DEADZONE
    );

    manualPivotTrigger.onTrue(
      new RunCommand(
        () -> {
          pivotSubsystem.setPower(-mechController.getLeftY());
        },
        pivotSubsystem
      )
    );

    manualPivotTrigger.onFalse(
      new RunCommand(
        () -> {
          pivotSubsystem.setPower(0);
        },
        pivotSubsystem
      )
    );

  }

  private void bindRollers(){

    rollerSubsystem.setDefaultCommand(new ConditionalCommand(
      new InstantCommand( () -> {
        rollerSubsystem.setRollerSpeed(.8 * ((mechController.getR2Axis() + 1.) / 2.)); 
      hasPiece = true;
      }, rollerSubsystem), 
      new InstantCommand( () -> {
        // System.out.println(hasPiece);
        rollerSubsystem.setRollerSpeed(.1 * (mechController.getR2Axis() - mechController.getL2Axis()));
      hasPiece = false;
      }, rollerSubsystem), 
      () -> rollerSubsystem.getCoralSensor() )); // 
    }

  //Binds the intake commands to the mech controller
  private void bindIntake(){
    bindPivotOverride();
    bindRollers();
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

    visionSubsystem1.setInterface(swerveSubsystem::addVisionMeasurements);
    visionSubsystem2.setInterface(swerveSubsystem::addVisionMeasurements);
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
    // try {
    //   driverCamera2 = new UsbCamera("fisheye2", 1);
    //   driverCamera2.setVideoMode(PixelFormat.kMJPEG, 160, 120, 30);
    //   driverCamera2.setExposureManual(40);
    // } catch (Exception e) {
    //   System.out.print(e);
    // }
  }

  public void constructNetworkTableListeners(){

    ntInstance = NetworkTableInstance.getDefault();
    FMStable = ntInstance.getTable("FMSInfo");
    testTable = ntInstance.getTable("testTable");

    //allianceEntry = FMStable.getEntry("IsRedAlliance");
    cameraSelectionEntry = testTable.getEntry("cameraSelection");
    // FMStable.addListener("IsRedAlliance", EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) -> {
    autonTableEntry = testTable.getEntry("autonList");
    autonTableSelection = testTable.getEntry("autonSelection");

    //     });
    testTable.addListener("cameraSelection", EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) ->{
      if (event.valueData.value.getBoolean()){
        //server1.setSource(camera1);
        driverCameraServer.setSource(driverCamera);

        System.out.println("CAMERA1!");
      }
      else{
        //server1.setSource(camera2);
        // driverCameraServer.setSource(driverCamera2);

        System.out.println("CAMERA2!");

      }
    });
    testTable.addListener("autonSelection", EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) ->{
      System.out.println("selectedAuton");
    });

  }



}
