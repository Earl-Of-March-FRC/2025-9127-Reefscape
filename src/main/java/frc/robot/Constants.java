// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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

  public static class DrivetrainConstants {
    //Relative to the centre in metres
    public static final Translation2d TOP_LEFT_POS = new Translation2d(0.26, 0.279);
    public static final Translation2d BOTTOM_LEFT_POS = new Translation2d(0.26, -0.279);
    public static final Translation2d TOP_RIGHT_POS = new Translation2d(-0.26, 0.279);
    public static final Translation2d BOTTOM_RIGHT_POS = new Translation2d(-0.26, -0.279);

    public static final double GEAR_RATIO = 10.71; 
    public static final int COUNTS_PER_REV = 4096;
    public static final double WHEEL_DIAMETER_INCHES = 6;

    public static final double COUNTS_TO_INCHES_CONVERSION = WHEEL_DIAMETER_INCHES*Math.PI/COUNTS_PER_REV/GEAR_RATIO;
    public static final double COUNTS_TO_METERS_CONVERSION = COUNTS_TO_INCHES_CONVERSION/39.37;

    //MOTOR ID'S
    public static final int TOP_LEFT_ID = 14;
    public static final int BOTTOM_LEFT_ID = 12;
    public static final int TOP_RIGHT_ID = 13;
    public static final int BOTTOM_RIGHT_ID = 11;

    public static final double SPEED_MULTIPLIER = 0.2;

    //"Empirical free speed" of a neo motor in RPM, from the manufacturer
    public static final double SIM_MAX_VELOCITY = COUNTS_PER_REV*5676;
  }

  public static class VisionConstants {
    //These are just some starting values, should tune these
    public static double kPX = 0.05;
    public static double kIX = 0;
    public static double kDX = 0;
    public static double kPY = 0.05;
    public static double kIY = 0;
    public static double kDY = 0;
    public static double kPA = 0.05;
    public static double kIA = 0;
    public static double kDA = 0;

  }
}