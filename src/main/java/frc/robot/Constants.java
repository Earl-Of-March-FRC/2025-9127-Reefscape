// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;

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
    public static final int kOperatorControllerPort = 1;
  }
  public static class ElevatorConstants{

    public static final double MANUAL_SPEED_MULTIPLIER = 0.2;

    //TODO verify gear ratio
    public static final double GEAR_RATIO = 10;

    //TODO confirm counts per rev, this may need to be changed to pulses per rev 
    public static final int COUNTS_PER_REV = 1;

    //TODO confirm that this is the correct sprocket diameter
    public static final double SPROCKET_DIAMETER_INCHES = 3.98; //NEEDS TESTING SINCE ITS NOT A WHEEL

    public static final double COUNTS_TO_INCHES_CONVERSION = SPROCKET_DIAMETER_INCHES * Math.PI / COUNTS_PER_REV / GEAR_RATIO;
    public static final double COUNTS_TO_METERS_CONVERSION = COUNTS_TO_INCHES_CONVERSION/39.37;

    public static final int LEADER_PORT = 6;
    public static final int FOLLOWER_PORT = 5;

    public static final double P_DOWN = 0;
    public static final double I_DOWN = 0;
    public static final double D_DOWN = 0;
    public static final double F_DOWN = 0;
    
    public static final double P_UP = 0;
    public static final double I_UP = 0;
    public static final double D_UP = 0;
    public static final double F_UP = 0;

    public static final ClosedLoopSlot PID_SLOT_DOWN = ClosedLoopSlot.kSlot0;
    public static final ClosedLoopSlot PID_SLOT_UP = ClosedLoopSlot.kSlot1;

    public static final double INTAKE_POSITION = 0;
    public static final double L1_POSITION= 0;
    public static final double L2_POSITION= 0;
    public static final double L3_POSITION = 0;
    public static final double L4_POSITION = 0;

    public static final int ENCODER_PORT_1 = 0;
    public static final int ENCODER_PORT_2 = 1;

    public static final int LOW_LIMIT_CHANNEL = 2;
    public static final int HIGH_LIMIT_CHANNEL = 3;

    //TODO mesure max position when elevator is fully extended
    public static final double MAX_POSITION = 0;

  }
}
