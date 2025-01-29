package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  private final NetworkTable limelightTable;
  private final NetworkTableEntry tv;
  private final NetworkTableEntry tx;
  private final NetworkTableEntry ty;
  private final NetworkTableEntry ta;
  private final NetworkTableEntry tshort;
  private final NetworkTableEntry tlong;
  private final NetworkTableEntry tid;

  private final double cameraHeight; // Camera height off the floor (meters)
  private final double cameraAngle;  // Camera pitch angle (degrees)
  private final double targetHeight;  // April tag height (meters)

  private double[] defaultTranslation = {0, 0, 0};
    private static final double tagWidth = 0.1524;

  //Constructor
  public VisionSubsystem(double cameraHeight, double cameraAngle, double targetHeight) {
      this.cameraHeight = cameraHeight;
      this.cameraAngle = cameraAngle;
      this.targetHeight = targetHeight;
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    tv = limelightTable.getEntry("tv");
    tx = limelightTable.getEntry("tx");
    ty = limelightTable.getEntry("ty");
    ta = limelightTable.getEntry("ta");
    tshort = limelightTable.getEntry("tshort");
    tlong = limelightTable.getEntry("tlong");
    tid = limelightTable.getEntry("tid");
  }

  //Returns if the limelight can see an object
  public boolean hasTarget() {
    return tv.getDouble(0) == 1;
  }

  public double getTargetID(){
    return tid.getDouble(0);
  }

  //Returns the Horizontal angle to the target (degrees)
  public double getHorizontalOffset() {
    return tx.getDouble(0);
  }

  //Returns the Vertical angle to the target (degrees)
    public double getVerticalOffset() {
    return ty.getDouble(0);
  }

  //Returns the Area of the target (percent)
  public double getTargetArea() {
    return ta.getDouble(0);
  }

    public double getShortSide(){
        return tshort.getDouble(0);
    }
    public double getLongSide(){
        return tlong.getDouble(0);
    }

    public double getDistanceToTag() {
        if (!hasTarget()){
            return -1;
        }
        return (targetHeight - cameraHeight) * 1/Math.tan(Math.toRadians(cameraAngle + getVerticalOffset()));

    }

    public double getXOffset(){
        if (!hasTarget()){
            return 0;
        }
        double distanceToTag = getDistanceToTag();
        return distanceToTag*Math.tan(Math.toRadians(getHorizontalOffset()));
    }
    
  @Override
  public void periodic() {
      SmartDashboard.putBoolean("Vision Has Target", hasTarget());
      SmartDashboard.putNumber("Vision Horizontal Offset", getHorizontalOffset());
      SmartDashboard.putNumber("Vision Vertical Offset", getVerticalOffset());
      SmartDashboard.putNumber("Vision Target Area", getTargetArea());
      SmartDashboard.putNumber("Distance to Tag", getDistanceToTag());
      SmartDashboard.putNumber("Vision Tag ID", getTargetID());
      SmartDashboard.putNumber("X Offset", getXOffset());
  }
}