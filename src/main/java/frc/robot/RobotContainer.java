// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AlignToTagCommand;
import frc.robot.commands.DriveFieldOriented;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final double cameraHeight = 0.0; //replace
  private final double cameraAngle = 0.0; //replace
  private final double targetHeight = 0.0; //replace

  private boolean fieldOriented;

  private final Drivetrain drivetrain = new Drivetrain();
  private final XboxController xboxController = new XboxController(0);
  private final VisionSubsystem vision = new VisionSubsystem(cameraHeight, cameraAngle, targetHeight);
  private final AlignToTagCommand alignToTag = new AlignToTagCommand(vision, drivetrain, 7); //Actual tag ID should be passed here

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    drivetrain.setDefaultCommand(new DriveFieldOriented(
      drivetrain, false,//xboxController.getLeftBumperButtonPressed(),
      () -> (xboxController.getLeftX()), 
      () -> -(xboxController.getLeftY()), 
      () -> (xboxController.getRightX())
      ));

    configureBindings();
  }


  private void configureBindings() {
    // Configure your button bindings here
    new JoystickButton(xboxController, XboxController.Button.kA.value)
            .onTrue(alignToTag);
    
    // new Trigger(xboxController::getLeftBumperButtonPressed).onTrue(new Command() {
    //   @Override
    //   public void execute() {
    //       // TODO Auto-generated method stub
    //       fieldOriented = !fieldOriented;
    //   }
    // });
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}