package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToTagCommand extends Command {

  private final VisionSubsystem visionSubsystem;
  private final Drivetrain driveSubsystem;
  private final int targetTagId;

  private double xTolerance = 0.1;
  private double yTolerance = 0.1;
  private double angleTolerance = 1;

  private final PIDController xController = new PIDController(Constants.VisionConstants.kPX, Constants.VisionConstants.kIX, Constants.VisionConstants.kDX);
  private final PIDController yController = new PIDController(Constants.VisionConstants.kPY, Constants.VisionConstants.kIY, Constants.VisionConstants.kDY);
  private final PIDController angleController = new PIDController(Constants.VisionConstants.kPA, Constants.VisionConstants.kIA, Constants.VisionConstants.kDA);

  private double targetXOffset;
  private double targetYOffset;
  private double targetAngleOffset;

  public AlignToTagCommand(VisionSubsystem visionSubsystem, Drivetrain driveSubsystem, int targetTagId) {
    this.targetTagId = targetTagId;
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem, visionSubsystem);

    // Configure PID controllers tuning will be critical here
    xController.setTolerance(xTolerance);
    yController.setTolerance(yTolerance);
    angleController.setTolerance(angleTolerance);
    angleController.enableContinuousInput(-180, 180);
  }

  @Override
  public void initialize() {

      targetXOffset = 0; //Set this to the desired xOffset of the robot
      targetYOffset = 0; //Set this to the desired Y offset of the robot
    targetAngleOffset = 0; //Sets the desired angle
    angleController.reset();
    xController.reset();
    yController.reset();
  }


    @Override
    public void execute() {
      
        if (!visionSubsystem.hasTarget()){
            driveSubsystem.drive(0,0,0, false);
            return;
        }
        double detectedTagId = visionSubsystem.getTargetID();
        SmartDashboard.putNumber("Detected AprilTag ID", detectedTagId);

        if (detectedTagId != targetTagId) {
            SmartDashboard.putString("Vision Debug", "Wrong Tag ID Detected!");
            driveSubsystem.drive(0, 0, 0, false);
            return;
      }
      
        double xOffset = visionSubsystem.getXOffset();
        double yOffset = visionSubsystem.getDistanceToTag();
        double angleOffset = visionSubsystem.getHorizontalOffset();
        double xSpeed = xController.calculate(xOffset, targetXOffset);
        double ySpeed = yController.calculate(yOffset, targetYOffset);
        double turnSpeed = angleController.calculate(angleOffset, targetAngleOffset);

        SmartDashboard.putNumber("Vision Error X", xController.getError());
        SmartDashboard.putNumber("Vision Error Y", yController.getError());
        SmartDashboard.putNumber("Vision Error Angle", angleController.getError());

      // Use the output of the PID controllers to set the robot's speed using mecanum drive
      //Adjust these 
      double maxSpeed = 1;
      double maxTurnSpeed = 0.5;

      //limit xSpeed and ySpeed to 1 without changing the ratio between the two
      double speedScaleDownFactor = 1;
      speedScaleDownFactor = Math.min(speedScaleDownFactor, maxSpeed/xSpeed);
      speedScaleDownFactor = Math.min(speedScaleDownFactor, maxSpeed/ySpeed);

      xSpeed = xSpeed*speedScaleDownFactor;
      ySpeed = ySpeed*speedScaleDownFactor;

      turnSpeed = MathUtil.clamp(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

      driveSubsystem.drive(xSpeed, ySpeed, turnSpeed, false);
  }
    @Override
    public boolean isFinished() {
      return xController.atSetpoint() && yController.atSetpoint() && angleController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0,0,0, false); //Stop when the command ends
    }
}