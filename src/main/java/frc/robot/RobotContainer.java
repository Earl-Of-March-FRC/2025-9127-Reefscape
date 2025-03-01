// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoRoutines.Routines.TimedRoutines.ExitZoneCommand;
import frc.robot.commands.AutoRoutines.Routines.ToReefScore.ScoreL1;
import frc.robot.commands.DriveFieldOriented;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final XboxController xboxController = new XboxController(0);
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();


    // Configure the trigger bindings
    drivetrain.setDefaultCommand(new DriveFieldOriented(
      drivetrain, xboxController.getLeftBumperButtonPressed(),
      () -> (xboxController.getLeftX()), 
      () -> (xboxController.getLeftY()), 
      () -> (xboxController.getRightX())
      ));

    configureBindings();

    //Go to Reef commands
    autoChooser.addOption("GO TO REEF FROM RIGHT", new PathPlannerAuto("To reef from right"));
    autoChooser.addOption("GO TO REEF FROM LEFT", new PathPlannerAuto("To reef from left"));
    autoChooser.addOption("GO TO REEF FROM CENTRE", new PathPlannerAuto("To reef from centre"));

    //Score on L1 Commands
    autoChooser.addOption("SCORE L1 FROM RIGHT", new ScoreL1(intakeSubsystem, "To reef from right"));
    autoChooser.addOption("SCORE L1 FROM LEFT", new ScoreL1(intakeSubsystem, "To reef from left"));
    autoChooser.addOption("SCORE L1 FROM CENTRE", new ScoreL1(intakeSubsystem, "To reef from centre"));

    //Exit Zone timed
    autoChooser.addOption("EXIT ZONE TIMED", new ExitZoneCommand(drivetrain, 1, 2));
    
    SmartDashboard.putData("Autonomous Routine", autoChooser);
  }

  private void configureBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
