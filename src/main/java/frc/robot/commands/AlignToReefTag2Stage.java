package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Command to align the robot to a reef using tx/ty values from Limelight
 */
public class AlignToReefTag2Stage extends Command {
    private final Drivetrain m_drive;
    private final LimelightSubsystem m_limelight;
    
    // PID controllers for alignment
    private final PIDController m_xController;  // Forward/back
    private final PIDController m_yController;  // Left/right (based on tx)
    private final PIDController m_rotationController; // Rotation (also based on tx)
    
    // Flags for alignment completion
    private boolean m_hasValidTarget = false;
    private boolean stage2 = false;
    
    // Tolerances
    private static final double TAG_X_TOLERANCE = 1.0;  // 1 degree tolerance
    private static final double TAG_Y_TOLERANCE = 1.0;  // 1 degree tolerance
    private static final double TX_TOLERANCE = 2.0; // 2 degrees

    //Setpoints
    private final double xOffset;
    private final double yOffset;

    private final double originalGyroOffset;
    
    /**
     * Creates a new direct alignment command using tx/ty values
     * 
     * @param driveSubsystem The robot's drive subsystem
     * @param limelightSubsystem The limelight subsystem
     */
    public AlignToReefTag2Stage(Drivetrain driveSubsystem, LimelightSubsystem limelightSubsystem, double xOffset, double yOffset) {
        m_drive = driveSubsystem;
        m_limelight = limelightSubsystem;

        this.xOffset = xOffset;
        this.yOffset = yOffset;

        //Store current angle adjustment,change angle adjustedment so that field oriented is relative to the tag (forward is towards the tag)
        originalGyroOffset = m_drive.getBotAngleAdjustment();
        m_drive.setBotAngleAdjustment(-m_limelight.getFilteredTargetXAngle());
        
        // Initialize PID controllers
        m_xController = new PIDController(VisionConstants.ALIGN_P_X, VisionConstants.ALIGN_I_X, VisionConstants.ALIGN_D_X);
        m_yController = new PIDController(VisionConstants.ALIGN_P_Y, VisionConstants.ALIGN_I_Y, VisionConstants.ALIGN_D_Y);
        m_rotationController = new PIDController(VisionConstants.ALIGN_P_ROT, VisionConstants.ALIGN_I_ROT, VisionConstants.ALIGN_D_ROT);
        
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

        // Set tolerances
        m_xController.setTolerance(TAG_X_TOLERANCE);
        m_yController.setTolerance(TAG_Y_TOLERANCE);
        m_rotationController.setTolerance(TX_TOLERANCE);

        // Set setpoints to align with middle (stage one)
        m_xController.setSetpoint(0);
        m_yController.setSetpoint(VisionConstants.DEFAULT_Y_OFFSET);
        m_rotationController.setSetpoint(0);
    }

    public void stage2() {
        // Set setpoints
        m_xController.setSetpoint(xOffset);
        m_yController.setSetpoint(yOffset);
        
        stage2 = true;
    }
    
    @Override
    public void execute() {
        // Check if we have a valid target
        m_hasValidTarget = m_limelight.hasValidTarget();
        
        if(isAligned() && !stage2){
            stage2();
        }
        
        if (m_hasValidTarget) {
            // Get filtered tx and ty values for stability

            //limelight coordinate space has different orientation from robot drive
            //Z+ limelight = Y+ robot drive
            //X+ limelight = X+ robot drive
            double currentTagX = m_limelight.getFilteredTagX();
            double currentTagY = m_limelight.getFilteredTagZ();
            double currentTx = m_limelight.getFilteredTargetXAngle();
            
            // Calculate motor outputs using PID controllers
            // Note: Signs may need to be adjusted based on your robot's coordinate system
            double xSpeed = -m_xController.calculate(currentTagX);  // Forward/back based on ty
            double ySpeed = -m_yController.calculate(currentTagY);  // Left/right based on tx
            double rotationSpeed = stage2? 0.0 : -m_rotationController.calculate(currentTx);  // Rotation to center target
            //m_rotationController.calculate(gyroHeading, wantedHeading);
            
            // Deemed not necessary
            // // Limit speeds for safety
            // xSpeed = MathUtil.clamp(xSpeed, -0.5, 0.5);
            // ySpeed = MathUtil.clamp(ySpeed, -0.5, 0.5);
            // rotationSpeed = MathUtil.clamp(rotationSpeed, -0.5, 0.5);
            
            // Drive the robot
            m_drive.driveFieldOriented(xSpeed, ySpeed, rotationSpeed);
            
            // Update dashboard
            SmartDashboard.putNumber("X Error", VisionConstants.DEFAULT_X_OFFSET - currentTagX);
            SmartDashboard.putNumber("Y Error", VisionConstants.DEFAULT_Y_OFFSET - currentTagY);
            SmartDashboard.putNumber("Tx Error", VisionConstants.DEFAULT_TX_OFFSET - currentTx);
            SmartDashboard.putBoolean("Alignment On Target", isAligned());
        } else {
            // No valid target found, stop the robot and turn until target is found
            m_drive.drive(0.0, 0.0, -m_limelight.lastTxSign()*0.2);
            SmartDashboard.putBoolean("Alignment On Target", false);
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        m_drive.drive(0.0, 0.0, 0.0);
        
        // Reset the angle adjustement
        m_drive.setBotAngleAdjustment(originalGyroOffset);

        if (interrupted) {
            System.out.println("TX/TY alignment interrupted");
        } else {
            System.out.println("TX/TY alignment completed successfully");
        }
    }
    
    @Override
    public boolean isFinished() {
        // Finish when aligned within tolerance
        return m_hasValidTarget && isAligned() && stage2;
    }
    
    /**
     * Check if the robot is aligned within tolerances
     */
    private boolean isAligned() {
        return m_xController.atSetpoint() && 
               m_yController.atSetpoint() && 
               (m_rotationController.atSetpoint() || stage2);
    }
}