package frc.robot.commands.AutoRoutines.Routines.ToReefScore;

import java.io.IOException;

import org.json.simple.parser.ParseException; // ADD THIS IMPORT

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.commands.AutoRoutines.Routines.AutoCommands.AlignAndScore;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.Drivetrain; // Import your command package
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem; // Import your subsystem package
import frc.robot.subsystems.LimelightSubsystem;


public class ThreePieceVision extends SequentialCommandGroup {

    /**
     * Autonomous routine to score 3 pieces on L4 using PathPlanner and Vision Alignment.
     * Assumes starting with one piece preloaded.
     */
    public ThreePieceVision(Drivetrain drivetrain, LimelightSubsystem limelight, Elevator elevator, IntakeSubsystem intakeSubsystem) {

        // Map<String, Command> eventMap = new HashMap<>();
        // eventMap.put("LowerToIntake", new ElevatorPID(elevator, ElevatorConstants.INTAKE_POSITION).withTimeout(2.0));
        // eventMap.put("RaiseToL4", new ElevatorPID(elevator, ElevatorConstants.L4_POSITION).withTimeout(4.0));
        // eventMap.put("StartIntake", new IntakeCommand(intakeSubsystem).withTimeout(AutoConstants.INTAKE_TIMEOUT_SECONDS));

        PathPlannerPath side1ToPickup = null;
        //PathPlannerPath pickupToSide2 = null;
        PathPlannerPath side2ToPickup = null;

        try {
            side1ToPickup = PathPlannerPath.fromPathFile("Side1ToPickup");
            //pickupToSide2 = PathPlannerPath.fromPathFile("Pickup2_To_L4_Score2");
            side2ToPickup = PathPlannerPath.fromPathFile("Side2ToPickup");

        } catch (IOException | ParseException e) {
            throw new RuntimeException("Failed to load PathPlanner paths.", e);
        }

        addCommands(
            //Piece 1//
            new AlignAndScore(drivetrain, limelight, elevator, intakeSubsystem),
            Commands.parallel(
                new ElevatorPID(elevator, ElevatorConstants.INTAKE_POSITION)
                // Commands.run(() -> drivetrain.driveRobotOriented(0, AutoConstants.AUTO_BACKUP_SPEED, 0), drivetrain)
                //         .withTimeout(AutoConstants.AUTO_BACKUP_DURATION)
            ),

            //Piece 2//
            AutoBuilder.followPath(side1ToPickup),
            new IntakeCommand(intakeSubsystem).withTimeout(AutoConstants.INTAKE_TIMEOUT_SECONDS),
            //AutoBuilder.followPath(pickupToSide2),
            new AlignAndScore(drivetrain, limelight, elevator, intakeSubsystem),
            Commands.parallel(
                new ElevatorPID(elevator, ElevatorConstants.INTAKE_POSITION)
                // Commands.run(() -> drivetrain.driveRobotOriented(0, AutoConstants.AUTO_BACKUP_SPEED, 0), drivetrain)
                //         .withTimeout(AutoConstants.AUTO_BACKUP_DURATION)
            ),

            //Piece 3//
            AutoBuilder.followPath(side2ToPickup),
            new IntakeCommand(intakeSubsystem).withTimeout(AutoConstants.INTAKE_TIMEOUT_SECONDS),
            //AutoBuilder.followPath(pickupToSide2),
            new AlignAndScore(drivetrain, limelight, elevator, intakeSubsystem)
        );
    }
}