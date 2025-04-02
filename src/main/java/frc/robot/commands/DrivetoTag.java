package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagVision;
import frc.robot.subsystems.Drivetrain;

public class DrivetoTag extends Command {
  private final Drivetrain drivetrain;
  private final AprilTagVision vision;
  private final String cameraName;
  private final double speed;
  private final double rotationKp = 0.02; // Proportional gain for rotation correction
  private final double positionKp = 0.5; // Proportional gain for forward/backward motion

  public DrivetoTag(Drivetrain drivetrain, AprilTagVision vision, String cameraName, double speed) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.cameraName = cameraName;
    this.speed = speed;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    Optional<Double> lateralDistance = vision.getLateralDistance(cameraName);
    Optional<Double> longitudinalDistance = vision.getLongitudinalDistance(cameraName);
    Optional<Double> tagAngle = vision.getTagHorizontalAngle(cameraName);

    if (longitudinalDistance.isEmpty() || lateralDistance.isEmpty() || tagAngle.isEmpty()) {
      System.out.println("NOTAG");
      drivetrain.drive(0, 0, 0); // Stop if can't see the tag
      return;
    }

    double forwardSpeed = -positionKp * longitudinalDistance.get(); // Drive forward/backward
    double strafeSpeed = -positionKp * lateralDistance.get(); // Strafe left/right
    double rotationSpeed = -rotationKp * tagAngle.get(); // Rotate towards the tag

    forwardSpeed = Math.max(-speed, Math.min(speed, forwardSpeed));
    strafeSpeed = Math.max(-speed, Math.min(speed, strafeSpeed));
    rotationSpeed = Math.max(-speed, Math.min(speed, rotationSpeed));

    drivetrain.drive(forwardSpeed, strafeSpeed, rotationSpeed);
  }

  @Override
  public boolean isFinished() {
    Optional<Double> longitudinalDistance = vision.getLongitudinalDistance(cameraName);
    return longitudinalDistance.isPresent() && Math.abs(longitudinalDistance.get()) < 0.2; // Stop when close enough
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0); 
  }
}
