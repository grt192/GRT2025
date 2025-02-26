// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.controllers.BaseDriveController;
import frc.robot.controllers.DualJoystickDriveController;
import frc.robot.controllers.PS5DriveController;
import frc.robot.controllers.XboxDriveController;

import java.util.EnumSet;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.FieldManagementSubsystem.FieldManagementSubsystem;
import frc.robot.subsystems.PhoenixLoggingSubsystem.PhoenixLoggingSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private BaseDriveController driveController;

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final FieldManagementSubsystem fieldManagementSubsystem = new FieldManagementSubsystem();
  private final PhoenixLoggingSubsystem phoenixLoggingSubsystem = new PhoenixLoggingSubsystem(fieldManagementSubsystem);
  
  //driver camera stuff:
  private UsbCamera driverCamera;
  private MjpegServer driverCameraServer;
  private UsbCamera driverCamera2;

  private NetworkTableInstance ntInstance;
  private NetworkTable testTable; 
  private NetworkTable FMStable;
  private NetworkTableEntry cameraSelectionEntry;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    constructDriveController(); 
    startLog();
    configureBindings();
    constructDriverCameras();
    constructNetworkTableListeners();

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
    return new PathPlannerAuto("3m Auto");
  }

  /**
   * Constructs the drive controller based on the name of the controller at port
   * 0
   */
  private void constructDriveController(){
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
    DataLogManager.start("/media/sda1/robotLogs");
    DriverStation.startDataLog(DataLogManager.getLog());

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
