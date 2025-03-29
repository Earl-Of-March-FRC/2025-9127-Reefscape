package frc.robot.commands.AutoRoutines.Routines.AutoCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.AlignToReefTagCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

// Helper command to raise elevator, align using vision, and shoot
public class AlignAndScore extends SequentialCommandGroup {

    public AlignAndScore(
            Drivetrain drivetrain,
            LimelightSubsystem limelight,
            Elevator elevator,
            IntakeSubsystem intakeSubsystem) {

        addCommands(
            
            // Align to the tag
            new AlignToReefTagCommand(
                drivetrain,
                limelight,
                VisionConstants.DEFAULT_X_OFFSET,
                VisionConstants.DEFAULT_Y_OFFSET,
                VisionConstants.DEFAULT_TX_OFFSET
            ).withTimeout(AutoConstants.ALIGNMENT_TIMEOUT_SECONDS), // Add timeout
            
            new RaiseAndShootCommand(intakeSubsystem, elevator, ElevatorConstants.L4_POSITION), // Raise and shoot
            
            // // Raise elevator to L4
            // Commands.deadline(
            //     Commands.waitUntil(() -> Math.abs(elevator.getPosition() - ElevatorConstants.L4_POSITION) < 1.0), // Wait until close
            //     new ElevatorPID(elevator, ElevatorConstants.L4_POSITION)
            // ),

            // // Shoot
            // Commands.deadline(
            //     Commands.waitSeconds(AutoConstants.AUTO_SHOOT_DURATION), // Shoot for a fixed duration
            //     new ShootCommand(intakeSubsystem, () -> AutoConstants.AUTO_SHOOT_SPEED)
            // ),

            Commands.waitSeconds(0.1)
        );
    }
}