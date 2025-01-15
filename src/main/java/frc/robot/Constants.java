// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

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
    public static class BackAlgaeConstants{
      public static final int ROLLER_MOTOR_ID = 1;//Placeholder
      public static final int PIVOT_MOTOR_ID = 2;//Placeholder
      public static final double PIVOT_P = 0.01;//placeholder value
      public static final double PIVOT_I = 0;//placeholder value
      public static final double PIVOT_D = 0;//placeholder value

      public static final int SENSOR_ID = 0;//placeholder value
      public static final double PIVOT_POSITION_1 = 0;//placeholder value
      public static final double PIVOT_POSITION_2 = 0;//placeholder value
    }
    public static class ClimbConstants{
      public static final int TOP_MOTOR_ID = 1;//Placeholder
      public static final int BOT_MOTOR_ID = 2;//Placeholder
      public static final double CLIMB_P = 0.01;//placeholder value
      public static final double CLIMB_I = 0;//placeholder value
      public static final double CLIMB_D = 0;//placeholder value
      public static final double CLIMB_POSITION_1 = 0;//placeholder value
      public static final double CLIMB_POSITION_2 = 0;//placeholder value
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
    public static final double MAX_VEL = 4200. / 6.923 / 60. * 2. * 2. * Math.PI * .0254; // Kraken speed / gear ratio / reduced to per second * circumference * convert to meters
    public static final double MAX_OMEGA = MAX_VEL / FL_POS.getNorm();

    public static final double DRIVE_P = 0.32; // temporary value
    public static final double DRIVE_I = 0; 
    public static final double DRIVE_D = 0;
    public static final double DRIVE_S = 0.1499;
    public static final double DRIVE_V = 0.112;

    public static final double STEER_P = 5.8;
    public static final double STEER_I = 0;
    public static final double STEER_D = 0;
    public static final double STEER_FF = 0;
  }

  public static class LoggingConstants{
    public static final String SWERVE_TABLE = "SwerveStats";
  }
}
