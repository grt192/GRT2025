// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.AutoAlignCommand;
import frc.robot.controllers.BaseDriveController;
import frc.robot.controllers.DualJoystickDriveController;
import frc.robot.controllers.PS5DriveController;
import frc.robot.controllers.XboxDriveController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.FieldManagementSubsystem.FieldManagementSubsystem;
import frc.robot.subsystems.PhoenixLoggingSubsystem.PhoenixLoggingSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import edu.wpi.first.wpilibj.PS5Controller;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.Commands.AutoAlignCommand;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private BaseDriveController driveController;
  private final CommandPS5Controller mechController = new CommandPS5Controller(0);


  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  //Bindings
  private final Trigger createTrigger;
  private final Trigger optionTrigger;
  private final Trigger xbutton;
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  private final FieldManagementSubsystem fieldManagementSubsystem =
    new FieldManagementSubsystem();

  // private final PhoenixLoggingSubsystem phoenixLoggingSubsystem =
    // new PhoenixLoggingSubsystem(fieldManagementSubsystem);


  private final SendableChooser<Command> autoChooser;

  private Trigger xButton;
  private Trigger squareButton;
  boolean isCompetition = false;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    
    constructDriveController(); 

    createTrigger = new Trigger(mechController.create());
    optionTrigger = new Trigger(mechController.options());
    xbutton = new Trigger(mechController.cross());
    
    startLog();
    configureBindings();

    mechController = new CommandPS5Controller(1);
    
    xButton = new Trigger(mechController.cross());
    squareButton = new Trigger(mechController.square());

    configureBindings();
    

    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
    (stream) -> isCompetition
      ? stream.filter(auto -> auto.getName().startsWith("C;"))
      : stream
    );
    SmartDashboard.putData("Auto Chooser", autoChooser);
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

      //HYPOTHETICAL BACK ALGAE CODE
      /*
       * Bind a button to move pivot into intaking position, rollers start rolling automatically
       *  Rollers stop when you press the button again or when distance  sensor detects ball
       * 
       * Bind a button to move pivot into outtaking position
       * Pressing that button twice spits the algae out
       */

    


    createTrigger.and(optionTrigger).whileTrue(
        new RunCommand(() -> {
          climbSubsystem.setSpeed(0.1);
          setRumble(1);
        }, climbSubsystem)
      ).onFalse(
        new RunCommand(() -> {
          climbSubsystem.setSpeed(0);

        }, climbSubsystem)
    );
  

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
    visionSubsystem.setInterface(swerveSubsystem::addVisionMeasurements);
    xButton.onTrue(
      AutoAlignCommand.reefTest(swerveSubsystem));

    visionSubsystem.setInterface(swerveSubsystem::addVisionMeasurements);
    squareButton.onTrue(
      AutoAlignCommand.sourceTest(swerveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("New Auto");
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
    driveController.setDeadZone(0.03);
  }

  /**
   * Starts datalog at /media/sda1/robotLogs
   */
  private void startLog(){
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }
}
