package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
/**
 * Subsystem for interacting with the Limelight vision system
 */
public class LimelightSubsystem extends SubsystemBase {
    // NetworkTable for the Limelight
    private final NetworkTable m_limelightTable;
    
    // Common Limelight entries
    private NetworkTableEntry m_tv;   // Whether limelight has valid target (0 or 1)
    private NetworkTableEntry m_tx;   // Horizontal offset from crosshair to target (-27 to 27 degrees)
    private NetworkTableEntry m_ty;   // Vertical offset from crosshair to target (-20.5 to 20.5 degrees)
    private NetworkTableEntry m_ta;   // Target area (0% to 100% of image)

    //Coordinate orrientation (bot centre is origin):
    // X+ is Right
    // Y+ is Down
    // Z+ is Forward (camera direction)
    private NetworkTableEntry m_botpose; // Target pose in robot coordinates (when using AprilTags)
    private NetworkTableEntry m_pipeline; // Current pipeline
    private NetworkTableEntry m_tid;    // AprilTag ID

    // Store the latest measurements for filtering
    private double m_lastTx = 0.0;
    private double m_lastTy = 0.0;
    private double m_lastTargetArea = 0.0;
    private double[] m_lastBotPose = new double[] {0.0, 0.0, 0.0};
    private int lastTxSign = 0;
    
    public LimelightSubsystem() {
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        
        // Initialize NetworkTable entries
        m_tv = m_limelightTable.getEntry("tv");
        m_tx = m_limelightTable.getEntry("tx");
        m_ty = m_limelightTable.getEntry("ty");
        m_ta = m_limelightTable.getEntry("ta");
        m_botpose = m_limelightTable.getEntry("targetpose_robotspace");
        m_pipeline = m_limelightTable.getEntry("pipeline");
        m_tid = m_limelightTable.getEntry("tid");
        
        // Set default pipeline to AprilTag detection
        setPipeline(VisionConstants.APRILTAG_PIPELINE);
        
        // Enable vision processing
        setLedMode(LedMode.OFF);
        setCameraMode(CameraMode.VISION);
    }
    
    @Override
    public void periodic() {
        // Get current values
        double currentTx = m_tx.getDouble(0.0);
        double currentTy = m_ty.getDouble(0.0);
        double currentArea = m_ta.getDouble(0.0);
        double[] currentBotPose = m_botpose.getDoubleArray(new double[] {0.0, 0.0, 0.0});
        
        // Apply simple filtering if we have a valid target
        if (hasValidTarget()) {
            // Simple low-pass filter
            double alpha = 0.2; // Filtering factor (adjust as needed)
            m_lastTx = alpha * currentTx + (1 - alpha) * m_lastTx;
            m_lastTy = alpha * currentTy + (1 - alpha) * m_lastTy;
            m_lastTargetArea = alpha * currentArea + (1 - alpha) * m_lastTargetArea;
            m_lastBotPose[0] = alpha * currentBotPose[0] + (1 - alpha) * m_lastBotPose[0];
            m_lastBotPose[1] = alpha * currentBotPose[1] + (1 - alpha) * m_lastBotPose[1];
            m_lastBotPose[2] = alpha * currentBotPose[2] + (1 - alpha) * m_lastBotPose[2];
        }

        if (Math.signum(getTargetXAngle()) != 0) {
            lastTxSign = (int) Math.signum(getTargetXAngle());
        }

        // Update dashboard with basic vision info
        SmartDashboard.putBoolean("Limelight Has Target", hasValidTarget());
        SmartDashboard.putNumber("Limelight Target X", getFilteredTargetXAngle());
        SmartDashboard.putNumber("Limelight Target Y", getFilteredTargetYAngle());
        SmartDashboard.putNumber("Limelight Target Area", getFilteredTargetArea());
        SmartDashboard.putNumber("AprilTag ID", getTargetID());
        SmartDashboard.putNumber("Tag X Raw", getTagX());
        SmartDashboard.putNumber("Tag Y Raw", getTagY());
        SmartDashboard.putNumber("Tag Z Raw", getTagZ());
        SmartDashboard.putNumber("Tag X", getFilteredTagX());
        SmartDashboard.putNumber("Tag Y", getFilteredTagY());
        SmartDashboard.putNumber("Tag Z", getFilteredTagZ());
    }
    
    public boolean hasValidTarget() {
        return m_tv.getDouble(0.0) > 0.5;
    }

    //return 1 if the most recent tx value was positive, -1 if it was negative values of exactly 0 will be ignored
    public int lastTxSign(){
        return lastTxSign;
    }
    
    /**
     * Get the raw horizontal offset from the crosshair to the target
     * @return Horizontal offset in degrees (-27 to 27)
     */
    public double getTargetXAngle() {
        return m_tx.getDouble(0.0);
    }
    
    /**
     * Get the filtered horizontal offset for more stable readings
     * @return Filtered horizontal offset in degrees
     */
    public double getFilteredTargetXAngle() {
        return hasValidTarget() ? m_lastTx : 0.0;
    }
    
    /**
     * Get the raw vertical offset from the crosshair to the target
     * @return Vertical offset in degrees (-20.5 to 20.5)
     */
    public double getTargetYAngle() {
        return m_ty.getDouble(0.0);
    }

    public double getTagX() {
        return m_botpose.getDoubleArray(new double[] {0.0, 0.0, 0.0})[0];
    }

    public double getFilteredTagX() {
        return hasValidTarget()? m_lastBotPose[0] : 0.0;
    }
    
    public double getTagY() {
        return m_botpose.getDoubleArray(new double[] {0.0, 0.0, 0.0})[1];
    }
    
    public double getFilteredTagY() {
        return hasValidTarget()? m_lastBotPose[1] : 0.0;
    }

    public double getTagZ() {
        return m_botpose.getDoubleArray(new double[] {0.0, 0.0, 0.0})[2];
    }

    public double getFilteredTagZ() {
        return hasValidTarget()? m_lastBotPose[2] : 0.0;
    }
    
    /**
     * Get the filtered vertical offset for more stable readings
     * @return Filtered vertical offset in degrees
     */
    public double getFilteredTargetYAngle() {
        return hasValidTarget() ? m_lastTy : 0.0;
    }
    
    /**
     * Get the raw area of the target
     * @return Target area as percentage of image (0 to 100)
     */
    public double getTargetArea() {
        return m_ta.getDouble(0.0);
    }
    
    /**
     * Get the filtered area of the target for more stable readings
     * @return Filtered target area
     */
    public double getFilteredTargetArea() {
        return hasValidTarget() ? m_lastTargetArea : 0.0;
    }
    
    /**
     * Get the ID of the detected AprilTag
     * @return AprilTag ID or -1 if none detected
     */
    public int getTargetID() {
        return (int) m_tid.getDouble(-1.0);
    }
    
    public void setPipeline(int pipeline) {
        m_pipeline.setNumber(pipeline);
    }
    
    public int getCurrentPipeline() {
        return (int) m_pipeline.getDouble(0.0);
    }
    
    public void setLedMode(LedMode mode) {
        m_limelightTable.getEntry("ledMode").setNumber(mode.getValue());
    }

    public void setCameraMode(CameraMode mode) {
        m_limelightTable.getEntry("camMode").setNumber(mode.getValue());
    }
    

    public enum LedMode {
        PIPELINE(0),  // Use LED mode from pipeline
        OFF(1),       // Force LEDs off
        BLINK(2),     // Force LEDs to blink
        ON(3);        // Force LEDs on
        
        private final int value;
        
        LedMode(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
    }
    
    public enum CameraMode {
        VISION(0),    // Vision processing mode
        DRIVER(1);    // Driver camera mode (optimized for driver viewing)
        
        private final int value;
        
        CameraMode(int value) {
            this.value = value;
        }
        
        public int getValue() {
            return value;
        }
    }
}
