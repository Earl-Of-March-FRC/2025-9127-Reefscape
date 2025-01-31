// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static int kOperatorControllerPort = 1;
  }
  public static class ElevatorConstants{

    public static final double GEAR_RATIO = 10.71; 
    public static final int COUNTS_PER_REV = 4096;
    public static final double INCHES_PER_REV = 6; //NEEDS TESTING SINCE ITS NOT A WHEEL

    public static final double COUNTS_TO_INCHES_CONVERSION = INCHES_PER_REV * Math.PI / COUNTS_PER_REV / GEAR_RATIO;

    public static final int MOTOR_PORT = 0;

    public static final int PID_SLOT = 0;
    public static final double P = 0;
    public static final double I = 0;
    public static final double D = 0;
    public static final double F = 0;

    public static final double INTAKE_POSITION = 0;
    public static final double L1_POSITION= 0;
    public static final double L2_POSITION= 0;
    public static final double L3_POSITION = 0;
    public static final double L4_POSITION = 0;

  }
}
