// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.controllers.BaseDriveController;
import frc.robot.controllers.DualJoystickDriveController;
import frc.robot.controllers.PS5DriveController;
import frc.robot.controllers.XboxDriveController;
import frc.robot.commands.Differential.DiffyTestPivot0Command;
import frc.robot.commands.Differential.DiffyTestPivot90Command;
import frc.robot.commands.Differential.DiffyTestWrist0Command;
import frc.robot.commands.Differential.DiffyTestWrist90Command;

import static frc.robot.Constants.SwerveConstants.BL_DRIVE;
import static frc.robot.Constants.SwerveConstants.BL_STEER;
import static frc.robot.Constants.SwerveConstants.BR_DRIVE;
import static frc.robot.Constants.SwerveConstants.BR_STEER;
import static frc.robot.Constants.SwerveConstants.FL_DRIVE;
import static frc.robot.Constants.SwerveConstants.FL_STEER;
import static frc.robot.Constants.SwerveConstants.FR_DRIVE;
import static frc.robot.Constants.SwerveConstants.FR_STEER;

import java.util.EnumSet;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DiffySubsystem.DiffyArmSubsystem;
import frc.robot.subsystems.DiffySubsystem.DiffyState;
import frc.robot.subsystems.FieldManagementSubsystem.FieldManagementSubsystem;
import frc.robot.subsystems.PhoenixLoggingSubsystem.PhoenixLoggingSubsystem;
import frc.robot.subsystems.swerve.SingleModuleSwerveSubsystem;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  //Controllers
  Trigger l1Trigger;
  Trigger r1Trigger;
  Trigger xButton;
  Trigger sButton;

  //Differential Drive Subsystems
  DiffyArmSubsystem diffyArmSubsystem;


  int offsetRads = 0;

  public double count;

  private final BaseDriveController driveController;

  // SwerveModule mod = new SwerveModule(BL_DRIVE, BL_STEER, offsetRads);
  // SingleModuleSwerveSubsystem singleModuleSwerve = new SingleModuleSwerveSubsystem(mod);
  // private final SwerveSubsystem swerveSubsystem;

  private final CommandPS5Controller mechController = new CommandPS5Controller(0);


  // SwerveModule mod = new SwerveModule(drivePort, steerPort, offsetRads);
  // IAmDyingSubsystem pls = new IAmDyingSubsystem();
  // SingleModuleSwerveSubsystem singleModuleSwerve = new SingleModuleSwerveSubsystem(mod);
  FieldManagementSubsystem fieldManagementSubsystem = new FieldManagementSubsystem();
  PhoenixLoggingSubsystem phoenixLoggingSubsystem = new PhoenixLoggingSubsystem(fieldManagementSubsystem);
  int state = 0;

  private NetworkTableInstance ntInstance;
  private NetworkTable swerveTable;
  private NetworkTableEntry swerveTestAngleEntry;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  private double thingy;  

  
  public RobotContainer() {
    
    // Configure controllers
    l1Trigger = new Trigger(mechController.L1());
    r1Trigger = new Trigger(mechController.R1());
    xButton = new Trigger(mechController.cross());
    sButton = new Trigger(mechController.square());

    // THIS WILL NEED TO BE CHANGED, THE BUTTONS AND ID

    // Configure Differential Drive Subsystems
    diffyArmSubsystem = new DiffyArmSubsystem();


    // swerveSubsystem = new SwerveSubsystem();
    
    driveController = new PS5DriveController();


    driveController.setDeadZone(0.03);

    ntInstance = NetworkTableInstance.getDefault();
    swerveTable = ntInstance.getTable("Swerve");
    swerveTestAngleEntry = swerveTable.getEntry("TestAngle");
    swerveTestAngleEntry.setDouble(0);
    // pls.configurePID(.5, 0, 0, 0); 
    // Configure the trigger bindings

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

      // l1Trigger.onTrue(
      //   new DifferentialTwistCommand(diffyArmSubsystem)
      // );


      // r1Trigger.onTrue(
      //   new DifferentialTurnCommand(diffyArmSubsystem)
      // );

      
      xButton.onTrue(
        new ConditionalCommand(
          new InstantCommand(() -> diffyArmSubsystem.setArmPosition(Units.degreesToRadians(90))), 
          new InstantCommand(() -> diffyArmSubsystem.setArmPosition(Units.degreesToRadians(0))), 
          () -> diffyArmSubsystem.atArmState(DiffyState.TEST_0))
        );

      // sButton.onTrue(
      //   new ConditionalCommand(
      //     new InstantCommand(() -> diffyArmSubsystem.setWristPosition(Units.degreesToRadians(90))), 
      //     new InstantCommand(() -> diffyArmSubsystem.setWristPosition(Units.degreesToRadians(0))), 
      //     () -> diffyArmSubsystem.atWristState(DiffyState.WTEST_0))
      //   );

        
      //   new DiffyTestPivot90Command(diffyArmSubsystem), 
      //   new DiffyTestPivot0Command(diffyArmSubsystem), 
      //   () -> diffyArmSubsystem.atArmState(DiffyState.TEST_0))
      // );

      // sButton.onTrue(new ConditionalCommand(
      //   new DiffyTestWrist90Command(diffyArmSubsystem), 
      //   new DiffyTestWrist0Command(diffyArmSubsystem), 
      //   () -> diffyArmSubsystem.atWristState(DiffyState.WTEST_0))
      // );
      


    
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

    // InstantCommand resetDriverHeadingCommand = new InstantCommand(() ->{ 
    //     swerveSubsystem.resetDriverHeading();
    //   },
    //   swerveSubsystem
    // );
    // /* Pressing the button resets the field axes to the current robot axes. */
    // driveController.bindDriverHeadingReset(
    //   () ->{
    //     swerveSubsystem.resetDriverHeading();
    //   },
    //   swerveSubsystem
    // );

    // swerveSubsystem.setDefaultCommand(new RunCommand(() -> {
    //       swerveSubsystem.setTestAngle(driveController.getForwardPower());
    //     }, swerveSubsystem
    // // ));

    // swerveTable.addListener("TestAngle", EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) -> {
    //   swerveSubsystem.setTestAngle(event.valueData.value.getDouble());
    // });

    // singleModuleSwerve.setDefaultCommand(new InstantCommand(() -> {
    //   System.out.println(mod.getWrappedAngle());
    //   if(driveController.getRightBumperButtonPressed()) {
    //     if (state == 7) {
    //       state = 0;
    //     }
    //     else {
    //       state += 1;
    //     }
    //   }

    //   switch (state) {
          // case 1: 
          //     singleModuleSwerve.setRawPowers(.5, 0);
          //     // singleModuleSwerve.setState(0, Units.degreesToRadians(180));
          //     break;

          // case 2:
          //     singleModuleSwerve.setRawPowers(-.5, 0);
          //     // singleModuleSwerve.setState(0, Units.degreesToRadians(180));
          //     break;

          // case 0:
          //     singleModuleSwerve.setRawPowers(0, .5);
          //     break;

          // case 4:
          //     singleModuleSwerve.setRawPowers(0, -.5);
          //     break;

          // case 5:
          //     singleModuleSwerve.setState(0, Units.degreesToRadians(45));
          //     break;

          // case 6:
          //     singleModuleSwerve.setState(0, Units.degreesToRadians(90));
          //     break;
          
          // case 7:
          //     singleModuleSwerve.setState(0, Units.degreesToRadians(135));
          //     break;
              
    //       case 3:
    //           singleModuleSwerve.setRawPowers(0, 0);
    //           break;

    //       default:
    //           break;
    //   }
    // }, singleModuleSwerve));

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
