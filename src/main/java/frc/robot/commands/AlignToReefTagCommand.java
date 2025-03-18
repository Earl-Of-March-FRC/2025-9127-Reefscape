package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstant;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Command to align the robot to a reef using tx/ty values from Limelight
 */
public class AlignToReefTagCommand extends Command {
    private final Drivetrain m_drive;
    private final LimelightSubsystem m_limelight;
    
    // PID controllers for alignment
    private final PIDController m_xController;  // Forward/back
    private final PIDController m_yController;  // Left/right (based on tx)
    private final PIDController m_rotationController; // Rotation (also based on tx)
    
    // Flags for alignment completion
    private boolean m_hasValidTarget = false;
    
    // Tolerances
    private static final double TAG_X_TOLERANCE = 1.0;  // 1 degree tolerance
    private static final double TAG_Y_TOLERANCE = 1.0;  // 1 degree tolerance
    private static final double TX_TOLERANCE = 2.0; // 2 degrees
    
    /**
     * Creates a new direct alignment command using tx/ty values
     * 
     * @param driveSubsystem The robot's drive subsystem
     * @param limelightSubsystem The limelight subsystem
     */
    public AlignToReefTagCommand(Drivetrain driveSubsystem, LimelightSubsystem limelightSubsystem) {
        m_drive = driveSubsystem;
        m_limelight = limelightSubsystem;
        
        // Initialize PID controllers
        m_xController = new PIDController(VisionConstants.ALIGN_P_X, VisionConstants.ALIGN_I_X, VisionConstants.ALIGN_D_X);
        m_yController = new PIDController(VisionConstants.ALIGN_P_Y, VisionConstants.ALIGN_I_Y, VisionConstants.ALIGN_D_Y);
        m_rotationController = new PIDController(VisionConstants.ALIGN_P_ROT, VisionConstants.ALIGN_I_ROT, VisionConstants.ALIGN_D_ROT);
        
        // Set tolerances
        m_xController.setTolerance(TAG_Y_TOLERANCE);
        m_yController.setTolerance(TAG_X_TOLERANCE);
        m_rotationController.setTolerance(TX_TOLERANCE);
        
        // Require subsystems
        addRequirements(m_drive, m_limelight);
    }
    
    @Override
    public void initialize() {
        // Switch to AprilTag pipeline
        m_limelight.setPipeline(VisionConstants.APRILTAG_PIPELINE);
        
        // Reset PID controllers
        m_xController.reset();
        m_yController.reset();
        m_rotationController.reset();

        // Set setpoints
        m_xController.setSetpoint(VisionConstant.X_SETPOINT_REEF_ALIGNMENT);
        m_yController.setSetpoint(VisionConstant.Y_SETPOINT_REEF_ALIGNMENT);
        m_rotationController.setSetpoint(0);
    }
    
    @Override
    public void execute() {
        // Check if we have a valid target
        m_hasValidTarget = m_limelight.hasValidTarget();
        
        if (m_hasValidTarget) {
            // Get filtered tx and ty values for stability
            double currentTagX = m_limelight.getFilteredTagX();
            double currentTagY = m_limelight.getFilteredTagY();
            double currentTx = m_limelight.getFilteredTargetXAngle();
            
            // Calculate motor outputs using PID controllers
            // Note: Signs may need to be adjusted based on your robot's coordinate system
            double xSpeed = m_xController.calculate(currentTagX);  // Forward/back based on ty
            double ySpeed = m_yController.calculate(currentTagY);  // Left/right based on tx
            double rotationSpeed = m_rotationController.calculate(currentTx);  // Rotation to center target
            //m_rotationController.calculate(gyroHeading, wantedHeading);
            
            // Limit speeds for safety
            xSpeed = MathUtil.clamp(xSpeed, -0.5, 0.5);
            ySpeed = MathUtil.clamp(ySpeed, -0.5, 0.5);
            rotationSpeed = MathUtil.clamp(rotationSpeed, -0.5, 0.5);
            
            // Drive the robot
            m_drive.driveRobotOriented(xSpeed, ySpeed, rotationSpeed);
            
            // Update dashboard
            SmartDashboard.putNumber("TX Error", VisionConstant.X_SETPOINT_REEF_ALIGNMENT - currentTagX);
            SmartDashboard.putNumber("TY Error", VisionConstant.Y_SETPOINT_REEF_ALIGNMENT - currentTagY);
            SmartDashboard.putBoolean("Alignment On Target", isAligned());
        } else {
            // No valid target found, stop the robot
            m_drive.drive(0.0, 0.0, 0.0);
            SmartDashboard.putBoolean("Alignment On Target", false);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        m_drive.drive(0.0, 0.0, 0.0);
        
        if (interrupted) {
            System.out.println("TX/TY alignment interrupted");
        } else {
            System.out.println("TX/TY alignment completed successfully");
        }
    }
    
    @Override
    public boolean isFinished() {
        // Finish when aligned within tolerance
        return m_hasValidTarget && isAligned();
    }
    
    /**
     * Check if the robot is aligned within tolerances
     */
    private boolean isAligned() {
        return m_xController.atSetpoint() && 
               m_yController.atSetpoint() && 
               m_rotationController.atSetpoint();
    }
}