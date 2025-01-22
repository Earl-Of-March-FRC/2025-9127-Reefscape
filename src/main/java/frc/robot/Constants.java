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
    //TODO Measure on the bot - used only for odemetry
    public static final Translation2d TOP_LEFT_POS = new Translation2d();
    public static final Translation2d BOTTOM_LEFT_POS = new Translation2d();
    public static final Translation2d TOP_RIGHT_POS = new Translation2d();
    public static final Translation2d BOTTOM_RIGHT_POS = new Translation2d();

    public static final double GEAR_RATIO = 10.71; 
    public static final int COUNTS_PER_REV = 42;
    public static final double WHEEL_DIAMETER_INCHES = 6;

    public static final double COUNTS_TO_INCHES_CONVERSION = WHEEL_DIAMETER_INCHES*Math.PI/COUNTS_PER_REV/GEAR_RATIO;
    public static final double COUNTS_TO_METERS_CONVERSION = COUNTS_TO_INCHES_CONVERSION/39.37;
  }
}
