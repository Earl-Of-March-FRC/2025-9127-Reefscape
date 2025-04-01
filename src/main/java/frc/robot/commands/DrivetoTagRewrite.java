package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AprilTagVision;
import frc.robot.subsystems.Drivetrain; 

public class DrivetoTagRewrite extends Command {

    private final Drivetrain drivetrain;
    private final AprilTagVision vision;
    private final String limelightName;
    private final double desiredDistanceMeters;

    // TODO tune these gains (start with p)
    private final PIDController rotationController = new PIDController(0.05, 0, 0.001); // P might be ~1/20th of max rotation speed per degree of error
    private final PIDController lateralController = new PIDController(1.5, 0, 0.05);    // P might be ~1-2 m/s per meter of error
    private final PIDController longitudinalController = new PIDController(1.5, 0, 0.05); // P might be ~1-2 m/s per meter of error

    // Tolerances - how close is close enough?
    private static final double ROTATION_TOLERANCE_DEGREES = 2.0;
    private static final double LATERAL_TOLERANCE_METERS = 0.05; // 5 cm
    private static final double LONGITUDINAL_TOLERANCE_METERS = 0.08;

    // State variable
    private boolean targetVisible = false;

    public DrivetoTagRewrite(Drivetrain drivetrain, AprilTagVision vision, String limelightName, double desiredDistanceMeters) {
        System.out.println("Starting...");
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.limelightName = limelightName;
        this.desiredDistanceMeters = desiredDistanceMeters;

        // Configure PID controllers
        rotationController.setTolerance(ROTATION_TOLERANCE_DEGREES);
        rotationController.setSetpoint(0.0); // Target angle is 0 (centered)

        lateralController.setTolerance(LATERAL_TOLERANCE_METERS);
        lateralController.setSetpoint(0.0); // Target lateral offset is 0

        longitudinalController.setTolerance(LONGITUDINAL_TOLERANCE_METERS);
        longitudinalController.setSetpoint(desiredDistanceMeters); // Target desired distance

        addRequirements(drivetrain, vision);

        SmartDashboard.putData("DriveToTag Rotation PID", rotationController);
        SmartDashboard.putData("DriveToTag Lateral PID", lateralController);
        SmartDashboard.putData("DriveToTag Longitudinal PID", longitudinalController);
    }

    @Override
    public void initialize() {
        System.out.println("DriveToAprilTag Initialized: Targeting distance " + desiredDistanceMeters + "m using " + limelightName);
        rotationController.reset();
        lateralController.reset();
        longitudinalController.reset();
        targetVisible = false;
    }

    @Override
    public void execute() {
        // Get the latest data from the vision subsystem
        Optional<Double> optAngle = vision.getTagHorizontalAngle(limelightName);
        Optional<Double> optLat = vision.getLateralDistance(limelightName);
        Optional<Double> optLon = vision.getLongitudinalDistance(limelightName);
        Optional<Integer> optId = vision.getTagId(limelightName); // Which tag were tacking

        // Check if the primary target is visible
        if (optAngle.isPresent() && optLat.isPresent() && optLon.isPresent() && optId.isPresent()) {
            targetVisible = true;
            double currentAngle = optAngle.get();
            double currentLateral = optLat.get();
            double currentLongitudinal = optLon.get();

            double rotationSpeed = -rotationController.calculate(currentAngle); // Negate if positive angle means target is left but positive rot is CCW
            double lateralSpeed = -lateralController.calculate(currentLateral);   // Negate if positive lateral means target is left but positive ySpeed is right
            double longitudinalSpeed = -longitudinalController.calculate(currentLongitudinal); // Negate if positive longitudinal means target is front but positive xSpeed is backward

            
            // rotationSpeed = MathUtil.clamp(rotationSpeed, -MAX_ROT_SPEED, MAX_ROT_SPEED);
            // lateralSpeed = MathUtil.clamp(lateralSpeed, -MAX_LAT_SPEED, MAX_LAT_SPEED);
            // longitudinalSpeed = MathUtil.clamp(longitudinalSpeed, -MAX_LON_SPEED, MAX_LON_SPEED);

            drivetrain.drive(longitudinalSpeed, lateralSpeed, rotationSpeed);

            Logger.recordOutput("DriveToTag/TargetID", optId.get());
            Logger.recordOutput("DriveToTag/RotationSpeed", rotationSpeed);
            Logger.recordOutput("DriveToTag/LateralSpeed", lateralSpeed);
            Logger.recordOutput("DriveToTag/LongitudinalSpeed", longitudinalSpeed);

        } else {
            targetVisible = false;
            drivetrain.drive(0, 0, 0);
            Logger.recordOutput("DriveToTag/TargetVisible", false);
        }
         Logger.recordOutput("DriveToTag/TargetVisible", targetVisible);
    }

    @Override
    public boolean isFinished() {
        // Command finishes if the target is visible AND all controllers are within tolerance
        return targetVisible &&
               rotationController.atSetpoint() &&
               lateralController.atSetpoint() &&
               longitudinalController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(0, 0, 0); // Stop
        System.out.println("DriveToAprilTag Ended. Interrupted: " + interrupted);
        Logger.recordOutput("DriveToTag/TargetVisible", false);
    }
}