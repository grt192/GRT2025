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
    }


    public static class DiffyConstants {
      public static final int LEFT_ID = 14;
      public static final int RIGHT_ID = 16;
      public static final double DIFFY_CONVERSION_FACTOR = (2 * Math.PI) / 30.;

      public static final double DIFFY_P = 1.5;
      public static final double DIFFY_I = 0;
      public static final double DIFFY_D = 0;

      public static final double ARM_ANGLE_180 = 0; //chabnge
      public static final double ARM_ANGLE_0 = 0; //change
      public static final double ARM_ANGLE_90 = 0; //change
      public static final double ARM_ANGLE_CORAL = 0; //change

      public static final double WRIST_ANGLE_180 = 0; //chabnge
      public static final double WRIST_ANGLE_0 = 0; //change
      public static final double WRIST_ANGLE_90 = 0; //change
      public static final double WRIST_ANGLE_CORAL = 0; //change

      public static final double DIFFY_TOLERANCE = 1; //change to .05

  }
}
