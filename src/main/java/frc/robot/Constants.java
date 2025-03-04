// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;

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
    public static final int kOperatorControllerPort = 1;
  }
  public static class ElevatorConstants{

    public static final double MANUAL_SPEED_MULTIPLIER = 0.2;

    //TODO verify gear ratio
    public static final double GEAR_RATIO = 10;

    //Motor controller automatically converts to revolutions
    public static final int COUNTS_PER_REV = 42;

    //TODO confirm that this is the correct sprocket diameter
    public static final double SPROCKET_TEETH = 15;
    public static final double CHAIN_PITCH = 3.0/8.0;


    public static final double COUNTS_TO_INCHES_CONVERSION = CHAIN_PITCH*SPROCKET_TEETH / GEAR_RATIO;
    public static final double COUNTS_TO_METERS_CONVERSION = COUNTS_TO_INCHES_CONVERSION/39.37;

    public static final int LEADER_PORT = 6;
    public static final int FOLLOWER_PORT = 5;

    public static final double P_DOWN = 1;
    public static final double I_DOWN = 0;
    public static final double D_DOWN = 0;
    public static final double F_DOWN = 0;
    
    public static final double P_UP = 1;
    public static final double I_UP = 0;
    public static final double D_UP = 0;
    public static final double F_UP = 0;

    public static final ClosedLoopSlot PID_SLOT_DOWN = ClosedLoopSlot.kSlot0;
    public static final ClosedLoopSlot PID_SLOT_UP = ClosedLoopSlot.kSlot1;

    public static final double INTAKE_POSITION = 0;
    public static final double L1_POSITION = 0;
    public static final double L2_POSITION = 0;
    public static final double L3_POSITION = 0;
    public static final double L4_POSITION = 0;

    public static final int ENCODER_PORT_1 = 0;
    public static final int ENCODER_PORT_2 = 1;

    public static final int LOW_LIMIT_CHANNEL = 2;
    public static final int HIGH_LIMIT_CHANNEL = 3;

    //TODO mesure max position when elevator is fully extended
    public static final double MAX_POSITION = 0;

  }

  public static class IntakeConstants {
    public static final int moto1pin = 0;
    public static final int moto2pin = 1;
    public static final int LIMIT_SWITCH_CHANNEL = 0;
    public static final double INTAKE_STOP_DELAY = 0.0; // TBD Change after testing
    public static final double SHOOT_MULTIPLIER = 0.5;
  }

  
  public static class DrivetrainConstants {

    //Relative to the centre in metres, used for kinematics and odometry0,\
    public static final Translation2d TOP_LEFT_POS = new Translation2d(0.26, 0.279);
    public static final Translation2d BOTTOM_LEFT_POS = new Translation2d(0.26, -0.279);
    public static final Translation2d TOP_RIGHT_POS = new Translation2d(-0.26, 0.279);
    public static final Translation2d BOTTOM_RIGHT_POS = new Translation2d(-0.26, -0.279);

    //Gearbox ratio
    public static final double GEAR_RATIO = 10.71; 

    public static final int COUNTS_PER_REV = 42;
    public static final double WHEEL_DIAMETER_INCHES = 6;

    //Conversion from native encoder units to wheel distance travelled (m)
    public static final double COUNTS_TO_INCHES_CONVERSION = WHEEL_DIAMETER_INCHES*Math.PI/COUNTS_PER_REV/GEAR_RATIO;
    public static final double COUNTS_TO_METERS_CONVERSION = COUNTS_TO_INCHES_CONVERSION/39.37;

    //Conversion from encoder RPM to wheel m/s
    public static final double RPM_TO_IPS_CONVERSION = WHEEL_DIAMETER_INCHES/60;
    public static final double RPM_TO_MPS_CONVERSION = RPM_TO_IPS_CONVERSION/39.37;

    //MOTOR ID'S
    public static final int TOP_LEFT_ID = 4;
    public static final int BOTTOM_LEFT_ID = 2;
    public static final int TOP_RIGHT_ID = 3;
    public static final int BOTTOM_RIGHT_ID = 1;

    //limits the max motor speed
    public static final double SPEED_MULTIPLIER = 1;

    //Controller deadband
    public static final double DRIVE_DEADBAND = 0.2;
    public static final double TURN_DEADBAND = 0.1;

    //"Empirical free speed" of a neo motor in RPM, from the manufacturer
    public static final double SIM_MAX_VELOCITY = COUNTS_PER_REV*5676;

    //Angle of the gyro's "zero yaw" position relative to the front of the bot
    public static final double GYRO_ANGLE_OFFSET = 0;
  }
}
