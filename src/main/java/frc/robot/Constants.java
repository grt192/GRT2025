// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Vision.CameraConfig;
import frc.robot.util.PolynomialRegression;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /** Constants for the swerve subsystem. */
  public static class SwerveConstants {
    public static final int FL_DRIVE = 0;
    public static final int FL_STEER = 1;
    public static final double FL_OFFSET = 0;

    public static final int FR_DRIVE = 2;
    public static final int FR_STEER = 3;
    public static final double FR_OFFSET = 0;

    public static final int BL_DRIVE = 4;
    public static final int BL_STEER = 5;
    public static final double BL_OFFSET = 0;

    public static final int BR_DRIVE = 6;
    public static final int BR_STEER = 7;
    public static final double BR_OFFSET = 0;

    public static double MODULE_DIST = Units.inchesToMeters(30 / 2.0);
    public static final Translation2d FL_POS = new Translation2d(MODULE_DIST, MODULE_DIST);
    public static final Translation2d FR_POS = new Translation2d(MODULE_DIST, -MODULE_DIST);
    public static final Translation2d BL_POS = new Translation2d(-MODULE_DIST, MODULE_DIST);
    public static final Translation2d BR_POS = new Translation2d(-MODULE_DIST, -MODULE_DIST);

    public static final double DRIVE_GEAR_REDUCTION = 9. * 20. / 26.;
    public static final double DRIVE_WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4 * Math.PI);
    public static final double MAX_VEL = 6000. / 6.923 / 60. * 2. * 2. * Math.PI * .0254; // Kraken speed / gear ratio / reduced to per second * circumference * convert to meters
    public static final double MAX_OMEGA = MAX_VEL / FL_POS.getNorm();

    public static final double[] DRIVE_P = new double[] {0.32, 0.32, 0.32, 0.32};
    public static final double[] DRIVE_I = new double[] {0, 0, 0, 0}; 
    public static final double[] DRIVE_D = new double[] {0, 0, 0, 0};
    public static final double[] DRIVE_S = new double[] {0.1499, 0.1499, 0.1499, 0.1499};
    public static final double[] DRIVE_V = new double[] {0.11, 0.112, 0.112, 0.112};

    public static final double[] STEER_P = new double[] {5.4, 5.4, 5.4, 5.4};
    public static final double[] STEER_I = new double[] {0, 0, 0, 0};
    public static final double[] STEER_D = new double[] {0, 0, 0, 0};
    public static final double[] STEER_FF = new double[] {0.036, 0.024, 0.0182, 0.05};
    
    public static final boolean DRIVE_DEBUG = false;
    public static final boolean STEER_DEBUG = false;
    public static final boolean STATE_DEBUG = false;
  }

  public static class ElevatorConstants {
    public static final double SOURCE= 0.0; //change
    public static final double L1 = 0.5; //change
    public static final double L2 = 0.0; //change
    public static final double L3 = 0.0; //change
    public static final double L4 = 0.0; //change
    public static final double GROUND = 0.0; 

    public static final double TOLERANCE = 8; //change

    public static final int KRAKEN_ID = 13; //change
    public static final int LIMIT_ID = 0; //change

    private static final double kP = 1; //change
    private static final double kI = 0; //change
    private static final double kD = 0; //change
    private static final double kG = 4; //chang
    public static final double kArbFF = 0;

    public static final double GEAR_RATIO = 20; //motor to axle
    public static final double AXLE_RADIUS = 6. * .289 * .0254; //in meters

    public static final double TICKS_TO_DIST = 2. * Math.PI * AXLE_RADIUS / GEAR_RATIO;
    public static final double DIST_TO_TICKS = 1. / TICKS_TO_DIST;

    public static final double dutyCycletoticks = 88.;
    
    private static final Slot0Configs slot0Configs = new Slot0Configs()
      .withKP(kP)
      .withKI(kI)
      .withKD(kD)
      .withKG(kG);
    private static final MotorOutputConfigs motorOutputConfigs =
      new MotorOutputConfigs()
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(InvertedValue.Clockwise_Positive);
    private static final SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs =
      new SoftwareLimitSwitchConfigs()
      .withReverseSoftLimitEnable(true)
      .withReverseSoftLimitThreshold(0)
      .withForwardSoftLimitEnable(true)
      .withForwardSoftLimitThreshold(88);
    public static TalonFXConfiguration TALON_CONFIG =
      new TalonFXConfiguration()
        .withSlot0(slot0Configs)
        .withMotorOutput(motorOutputConfigs)
        .withSoftwareLimitSwitch(softwareLimitSwitchConfigs);

    public static final boolean DEBUG = true;
  }

  public static class LoggingConstants{
    public static final String SWERVE_TABLE = "SwerveStats";
    public static final String REV_TABLE = "MotorStats";
    public static final String CTRE_TABLE = "MotorStats";
  }

  public static class DebugConstants{
    public static final boolean STATE_DEBUG = false;
    public static final boolean DRIVE_DEBUG = false;
    public static final boolean STEER_DEBUG = false;
    public static final boolean REV_DEBUG = false;
    public static final boolean CTRE_DEBUG = false;
  }

  public static class VisionConstants{

    public static final double[] STD_DEV_DIST = new double[] {
      0.75, 1.00, 1.3, 1.69, 2., 2.51, 2.78, 3.07, 3.54, 4.1, 4.52 
    };

    public static final double[] X_STD_DEV = new double[] {
      0.002, 0.005, 0.007, 0.014, 0.029, 0.074, 0.101, 0.12, 0.151, 0.204, 0.287
    };

    public static final double[] Y_STD_DEV = new double[] {
      0.002, 0.005, 0.013, 0.020, 0.067, 0.080, 0.095, 0.160, 0.206, 0.259, 0.288
    };

    public static final double[] O_STD_DEV = new double[] {
      0.002, 0.004, 0.005, 0.011, 0.031, 0.4, 1.72, 1.89, 2.05, 2.443, 2.804
    };

    public static final Pose3d[] CAMERA_POSES = new Pose3d[] {
      new Pose3d(0.20, 0, 0.20, new Rotation3d(0, 0, 0))
    };

    public static final CameraConfig[] cameraConfigs = new CameraConfig[]{
      new CameraConfig(
        "1",
        new Transform3d(
          0.31, 0.01, 0.2,
          new Rotation3d(- Math.PI / 2., 0, 0)
        ),
        PoseStrategy.LOWEST_AMBIGUITY
      ),
      new CameraConfig(
        "2",
        new Transform3d(
          -0.19, -0.065, 0.2,
          new Rotation3d(Math.PI / 2., 0., Math.PI)
        ),
        PoseStrategy.LOWEST_AMBIGUITY
      )
    };
    public static final PolynomialRegression xStdDevModel = new PolynomialRegression(
      VisionConstants.STD_DEV_DIST,VisionConstants.X_STD_DEV,2);
    public static final PolynomialRegression yStdDevModel = new PolynomialRegression(
      VisionConstants.STD_DEV_DIST,VisionConstants.Y_STD_DEV,2);
    public static final PolynomialRegression oStdDevModel = new PolynomialRegression(
      VisionConstants.STD_DEV_DIST,VisionConstants.O_STD_DEV,1);
  }
}
