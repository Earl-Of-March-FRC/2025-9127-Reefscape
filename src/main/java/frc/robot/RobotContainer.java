// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.ManualElevator;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveFieldOriented;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ReverseCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.IntakeSubsystem;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final XboxController driveController = new XboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final Elevator elevator = new Elevator();
  private final ElevatorPID[] elevatorCommands;
  private int elevatorPositionIndex;

  private final IntakeSubsystem intakeSub = new IntakeSubsystem();

  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
public RobotContainer() {
    intakeSub.setDefaultCommand(
        new ShootCommand(
            intakeSub,
            () -> operatorController.getLeftTriggerAxis()
        )
    );

    elevatorPositionIndex = 0;
    elevatorCommands = new ElevatorPID[]{
      new ElevatorPID(elevator, Constants.ElevatorConstants.INTAKE_POSITION),
      new ElevatorPID(elevator, Constants.ElevatorConstants.L1_POSITION),
      new ElevatorPID(elevator, Constants.ElevatorConstants.L2_POSITION),
      new ElevatorPID(elevator, Constants.ElevatorConstants.L3_POSITION),
      new ElevatorPID(elevator, Constants.ElevatorConstants.L4_POSITION)
    };

    // Configure the trigger bindings
    drivetrain.setDefaultCommand(new DriveFieldOriented(
      drivetrain,
      () -> (driveController.getLeftX()), //X translation
      () -> -(driveController.getLeftY()), //Y translation
      () -> (driveController.getRightX()) //Z rotation
      ));
    configureBindings();
    elevator.setDefaultCommand(new ManualElevator(elevator, ()-> -m_operatorController.getRightTriggerAxis() + m_operatorController.getLeftTriggerAxis()));
  }

  private void configureBindings() {
    // Configure your button bindings here

    //Reset the gyro angle to 0 when A is pressed on the driver controller
    new Trigger(driveController::getAButtonPressed).onTrue(Commands.runOnce(() -> drivetrain.resetGyro(), drivetrain));
    
    //Toggle the drive mode (field or robot oriented) when B is pressed on the driver controller
    new Trigger(driveController::getBButtonPressed).onTrue(Commands.runOnce(() -> drivetrain.changeDriveMode(), drivetrain));

    //automatically intake with beam break sensor using button a
    operatorController.a().whileTrue(new IntakeCommand(intakeSub));

    //reverse direction for intake with right trigger
    new Trigger(() -> operatorController.getRightTriggerAxis() > 0.1)
        .whileTrue(new ReverseCommand(
            intakeSub,
            () -> operatorController.getRightTriggerAxis()
        ));

    operatorController.leftBumper().onTrue(new InstantCommand(()->{
      elevatorPositionIndex = (elevatorPositionIndex + 1) % elevatorCommands.length;
      elevatorCommands[elevatorPositionIndex].schedule();
    }));
    
    operatorController.rightBumper().onTrue(new InstantCommand(()->{
      elevatorPositionIndex = (elevatorPositionIndex - 1 + elevatorCommands.length) % elevatorCommands.length;
      elevatorCommands[elevatorPositionIndex].schedule();
    }));

  }

  public Command getAutonomousCommand() {
    return null;
  }
}