package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Command to align the robot to a reef using tx/ty values from Limelight
 */
public class AlignToReefTxTyCommand extends Command {
    private final Drivetrain m_drive;
    private final LimelightSubsystem m_limelight;
    
    // PID controllers for alignment
    private final PIDController m_xController;  // Forward/back
    private final PIDController m_yController;  // Left/right (based on tx)
    private final PIDController m_rotationController; // Rotation (also based on tx)
    
    // Target values
    private final double m_targetTy;  // Target ty value (controls distance)
    private final double m_targetTx;  // Target tx value (usually 0 for centered)
   // private final double m_targetArea; // Target area (optional, for distance)
    
    // Flags for alignment completion
    private boolean m_hasValidTarget = false;
    
    // Tolerances
    private static final double TX_TOLERANCE = 1.0;  // 1 degree tolerance
    private static final double TY_TOLERANCE = 1.0;  // 1 degree tolerance
    private static final double ROTATION_TOLERANCE = 2.0; // 2 degrees
    
    /**
     * Creates a new direct alignment command using tx/ty values
     * 
     * @param driveSubsystem The robot's drive subsystem
     * @param limelightSubsystem The limelight subsystem
     * @param targetTy The target ty value (controls distance from target)
     * @param targetTx The target tx value (controls lateral alignment, usually 0)
     */
    public AlignToReefTxTyCommand(Drivetrain driveSubsystem, LimelightSubsystem limelightSubsystem, double targetTy, double targetTx) {
        m_drive = driveSubsystem;
        m_limelight = limelightSubsystem;
        m_targetTy = targetTy;
        m_targetTx = targetTx;
       // m_targetArea = 0.0; // Not using area in this implementation
        
        // Initialize PID controllers
        m_xController = new PIDController(VisionConstants.ALIGN_P_X, VisionConstants.ALIGN_I_X, VisionConstants.ALIGN_D_X);
        m_yController = new PIDController(VisionConstants.ALIGN_P_Y, VisionConstants.ALIGN_I_Y, VisionConstants.ALIGN_D_Y);
        m_rotationController = new PIDController(VisionConstants.ALIGN_P_ROT, VisionConstants.ALIGN_I_ROT, VisionConstants.ALIGN_D_ROT);
        
        // Set tolerances
        m_xController.setTolerance(TY_TOLERANCE);
        m_yController.setTolerance(TX_TOLERANCE);
        m_rotationController.setTolerance(ROTATION_TOLERANCE);
        
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
        
        System.out.println("Starting tx/ty alignment with targets: tx=" + m_targetTx + ", ty=" + m_targetTy);
    }
    
    @Override
    public void execute() {
        // Check if we have a valid target
        m_hasValidTarget = m_limelight.hasValidTarget();
        
        if (m_hasValidTarget) {
            // Get filtered tx and ty values for stability
            double currentTx = m_limelight.getFilteredTargetX();
            double currentTy = m_limelight.getFilteredTargetY();
            
            // Calculate errors
            double tyError = currentTy - m_targetTy;  // Error in vertical angle (distance)
            double txError = currentTx - m_targetTx;  // Error in horizontal angle (alignment)
            
            // Calculate motor outputs using PID controllers
            // Note: Signs may need to be adjusted based on your robot's coordinate system
            double xSpeed = -m_xController.calculate(tyError, 0.0);  // Forward/back based on ty
            double ySpeed = -m_yController.calculate(txError, 0.0);  // Left/right based on tx
            double rotationSpeed = -m_rotationController.calculate(txError, 0.0);  // Rotation to center target
            //m_rotationController.calculate(gyroHeading, wantedHeading);
            
            // Limit speeds for safety
            xSpeed = clamp(xSpeed, -0.5, 0.5);
            ySpeed = clamp(ySpeed, -0.5, 0.5);
            rotationSpeed = clamp(rotationSpeed, -0.5, 0.5);
            
            // Drive the robot
            m_drive.drive(xSpeed, ySpeed, rotationSpeed, true);
            
            // Update dashboard
            SmartDashboard.putNumber("TX Error", txError);
            SmartDashboard.putNumber("TY Error", tyError);
            SmartDashboard.putBoolean("Alignment On Target", isAligned());
        } else {
            // No valid target found, stop the robot
            m_drive.drive(0.0, 0.0, 0.0, true);
            SmartDashboard.putBoolean("Alignment On Target", false);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        m_drive.drive(0.0, 0.0, 0.0, true);
        
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
    
    /**
     * Utility method to clamp a value between a min and max
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}