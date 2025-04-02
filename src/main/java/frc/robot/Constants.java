// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.geometry.Pose2d;
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

  // public static class VisionConstant {
  //   public static final double X_REEF_ALIGNMENT_P = 0;
  //   public static final double Y_REEF_ALIGNMENT_P = 0;
  //   public static final double Z_REEF_ALIGNMENT_P = 0;

  //   public static final double Z_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
  //   public static final double Z_TOLERANCE_REEF_ALIGNMENT = 1;
  //   public static final double X_SETPOINT_REEF_ALIGNMENT = -0.34;  // Vertical pose
  //   public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;
  //   public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16;  // Horizontal pose
  //   public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

  //   public static final double DONT_SEE_TAG_WAIT_TIME = 1;
  //   public static final double POSE_VALIDATION_TIME = 0.3;
  // }
  
  public static class VisionConstants {
    // Limelight pipeline indices
    public static final int APRILTAG_PIPELINE = 0;  // Pipeline configured for AprilTag detection
    public static final int DRIVER_PIPELINE = 1;    // Pipeline configured for driver camera
    
    // Camera mounting position (THESE NEED TO BE CALIBRATED)
    public static final double CAMERA_HEIGHT_METERS = 0.5;  // Height of camera from floor
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(15.0);  // Camera angle from horizontal
    
    // Target properties
    public static final double TARGET_HEIGHT_METERS = 0.6;  // Height of AprilTag center from floor
    
    // PID constants for alignment
    // These will need tuning on your actual robot
    public static final double ALIGN_P_X = 0.8;
    public static final double ALIGN_I_X = 0.1;
    public static final double ALIGN_D_X = 0.0;
    
    public static final double ALIGN_P_Y = 0.8;
    public static final double ALIGN_I_Y = 0.1;
    public static final double ALIGN_D_Y = 0.0;
    
    public static final double ALIGN_P_ROT = 0.01;
    public static final double ALIGN_I_ROT = 0.0;
    public static final double ALIGN_D_ROT = 0.0;

    // Default offsets for reef alignment (adjust based on game piece handling)
    //NOTE: this translation must be the same as the limelight 3D POI offset in hardware client for Tx values to be accurate
    public static final double DEFAULT_X_OFFSET = -0.1296561;  // 75cm from the reef (adjust as needed)
    public static final double DEFAULT_Y_OFFSET = 0.7064591;  // Centered laterally
    public static final double DEFAULT_TX_OFFSET = -17.30; // Square to target
  }
  public final class AutoConstants {

    public static final double AUTO_SHOOT_SPEED = 0.55; 
    public static final double AUTO_SHOOT_DURATION = 1.0; 

    public static final double AUTO_BACKUP_SPEED = -0.3; 
    public static final double AUTO_BACKUP_DURATION = 0.75;

    public static final double ALIGNMENT_TIMEOUT_SECONDS = 3.0;
    public static final double INTAKE_TIMEOUT_SECONDS = 2.0;
    public static final Pose2d LEFT_STATION_POSE = null;
    public static final Pose2d RIGHT_STATION_POSE = null;
}

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }
  public static class ElevatorConstants{

    public static final double SLOW_MODE_MULTIPLIER = 0.5;

    public static final double MANUAL_SPEED_MULTIPLIER = 0.3;
    public static final double MANUAL_OFFSET = 0.5;

    //TODO verify gear ratio
    public static final double GEAR_RATIO = 10;

    //Motor controller automatically converts to revolutions
    public static final int COUNTS_PER_REV = 42;

    //TODO confirm that this is the correct sprocket diameter
    public static final double SPROCKET_TEETH = 15;
    public static final double CHAIN_PITCH = 3.0/8.0;


    public static final double COUNTS_TO_INCHES_CONVERSION = 2*CHAIN_PITCH*SPROCKET_TEETH/GEAR_RATIO;
    public static final double COUNTS_TO_METERS_CONVERSION = COUNTS_TO_INCHES_CONVERSION/39.37;

    public static final int LEADER_PORT = 6;
    public static final int FOLLOWER_PORT = 5;

    public static final double P_DOWN = 0.02;
    public static final double I_DOWN = 0.0;
    public static final double D_DOWN = 0;
    public static final double F_DOWN = 0;
    
    public static final double P_UP = 0.11;
    public static final double I_UP = 0.002;
    public static final double D_UP = 0;
    public static final double F_UP = 0;

    public static final ClosedLoopSlot PID_SLOT_DOWN = ClosedLoopSlot.kSlot0;
    public static final ClosedLoopSlot PID_SLOT_UP = ClosedLoopSlot.kSlot1;

    public static final double MAX_PID_OUTPUT_UP = 0.5;
    public static final double MAX_PID_OUTPUT_DOWN = 0.5;

    public static final double SETPOINT_OFFSET = 4;
    public static final double MANIPULATOR_BASE_OFF_GROUND_INCHES = 12.5;
    public static final double INTAKE_POSITION = 0;
    public static final double L1_POSITION = (26 - MANIPULATOR_BASE_OFF_GROUND_INCHES) + SETPOINT_OFFSET;
    public static final double L2_POSITION = (31.875 - MANIPULATOR_BASE_OFF_GROUND_INCHES) + SETPOINT_OFFSET;
    public static final double L3_POSITION = (47.625 - MANIPULATOR_BASE_OFF_GROUND_INCHES) + SETPOINT_OFFSET;
    public static final double L4_POSITION = (72 - MANIPULATOR_BASE_OFF_GROUND_INCHES) + SETPOINT_OFFSET;

    //TODO: calibrate these values

    //Offset that allows the algae remover to grip the ball
    public static final double ALGAE_OFFSET = 2;
    public static final double L3_ALGAE_POSITION = L3_POSITION + ALGAE_OFFSET;
    public static final double L2_ALGAE_POSITION = L2_POSITION + ALGAE_OFFSET;

    //Postion that allows algae removal to fall out
    public static final double ALGAE_RELEASE_POSITION = 0;


    //Tolerance for elevator position that needs to be achieved before servo can be activated
    public static final double TOLERANCE = 1;

    public static final int LOW_LIMIT_CHANNEL = 2;
    public static final int HIGH_LIMIT_CHANNEL = 3;

    //TODO mesure max position when elevator is fully extended
    public static final double MAX_POSITION = 0;
  }

  public static class IntakeConstants {
    public static final int moto1pin = 7;
    public static final int moto2pin = 8;
    public static final int LIMIT_SWITCH_CHANNEL = 0;
    public static final double INTAKE_STOP_DELAY = 0.082  ; // TBD Change after testing
    public static final double SHOOT_MULTIPLIER = 0.5;
  }
  
  public static class DrivetrainConstants {

    //TODO: calibrate these values
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4; //Max net robot translational speed
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kMaxAccelerationMetersPerSecondSquaredPathfinding = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

    public static final PathConstraints kPathfindingConstraints = new PathConstraints(kMaxSpeedMetersPerSecond,
        kMaxAccelerationMetersPerSecondSquaredPathfinding, kMaxAngularSpeedRadiansPerSecond,
        kMaxAngularAccelerationRadiansPerSecondSquared);

    //TODO: calibrate these values
    public static final double TRACK_WIDTH = 0.53; //Distance between the wheels in metres
    public static final double WHEEL_BASE = 0.53; //Distance between the front and back wheels in metres

    public static final double CHASSIS_WIDTH = Units.inchesToMeters(26.5); //Chassis width in metres
    public static final double CHASSIS_LENGTH = Units.inchesToMeters(32.31); //Chassis length in metres

    public static final double BUMPER_WIDTH = Units.inchesToMeters(3.5); //Bumper width in metres

    public static final double ROBOT_WIDTH = CHASSIS_WIDTH + BUMPER_WIDTH; //Chassis width in metres +bumpers
    public static final double ROBOT_LENGTH = CHASSIS_LENGTH + BUMPER_WIDTH; //Chassis length in metres + bumpers

    //Relative to the centre in metres, used for kinematics and odometry0,\
    public static final Translation2d TOP_LEFT_POS = new Translation2d(0.26, 0.279);
    public static final Translation2d BOTTOM_LEFT_POS = new Translation2d(0.26, -0.279);
    public static final Translation2d TOP_RIGHT_POS = new Translation2d(-0.26, 0.279);
    public static final Translation2d BOTTOM_RIGHT_POS = new Translation2d(-0.26, -0.279);

    //Gearbox ratio
    public static final double GEAR_RATIO = 10.71; 
    
    //Motor controller automatically converts to revolutions
    public static final int COUNTS_PER_REV = 42;
    public static final double WHEEL_DIAMETER_INCHES = 6;

    //Conversion from native encoder units to wheel distance travelled (m)
    public static final double COUNTS_TO_INCHES_CONVERSION = WHEEL_DIAMETER_INCHES*Math.PI/GEAR_RATIO;
    public static final double COUNTS_TO_METERS_CONVERSION = COUNTS_TO_INCHES_CONVERSION/39.37;
    //PID CONSTANTS
    public static final double TRANSLATE_P = 0.1;
    public static final double TRANSLATE_I = 0.0;
    public static final double TRANSLATE_D = 0.0;

    public static final double ROTATE_P = 0.1;
    public static final double ROTATE_I = 0.0;
    public static final double ROTATE_D = 0.0;

    public static final double PATHPLANNER_TRANSLATE_P = 0.03;
    public static final double PATHPLANNER_TRANSLATE_I = 0.0;
    public static final double PATHPLANNER_TRANSLATE_D = 0.0;

    public static final double PATHPLANNER_ROTATE_P = 0.1;
    public static final double PATHPLANNER_ROTATE_I = 0.0;
    public static final double PATHPLANNER_ROTATE_D = 0.0;

    public static final double VELOCITY_KP = 0.07;
    public static final double VELOCITY_KI = 0.0;
    public static final double VELOCITY_KD = 0.0;
    
    //Feedfoward gain for velocity control
    //473 is the NEO Kv, as supplied by the manufacturer
    //0.072
    public static final double VELOCITY_Kf = 0.08;

    //MOTOR ID'S
    public static final int TOP_LEFT_ID = 4;
    public static final int BOTTOM_LEFT_ID = 2;
    public static final int TOP_RIGHT_ID = 3;
    public static final int BOTTOM_RIGHT_ID = 1;

    //limits the max motor speed
    public static final double SPEED_MULTIPLIER = 0.9;
    public static final double SLOW_SPEED_MULTIPLIER = 0.4;
    public static final double PID_SPEED_MULTIPLIER = 0.8;

    //Controller deadband
    public static final double DRIVE_DEADBAND = 0.2;
    public static final double TURN_DEADBAND = 0.1;
    
    //Conversion from encoder RPM to wheel m/s
    public static final double RPM_TO_IPS_CONVERSION = WHEEL_DIAMETER_INCHES/60;
    public static final double RPM_TO_MPS_CONVERSION = RPM_TO_IPS_CONVERSION/39.37;

    //"Empirical free speed" of a neo motor in RPM, from the manufacturer
    public static final double SIM_MAX_VELOCITY = 5676;
    //Max wheel speed in metres per second, uses the above mentioned NEO free speed, our wheel of course have load on them so we reduce the speed by 20%
    //TODO: calibrate this value further
    public static final double MAX_SPEED_MPS = (5676 * RPM_TO_MPS_CONVERSION / GEAR_RATIO) * 0.8;
 
    //Angle of the gyro's "zero yaw" position relative to the front of the bot
    public static final double GYRO_ANGLE_OFFSET = 0;
  }
  public static class AlgaeRemovalConstants {
    public static final int SERVO_PORT = 1;

    //TODO: calibrate these values based on servo orientation (0-1)
    public static final double UP_POSITION = 1;
    public static final double DOWN_POSITION = 0.3;
    

  }
}
