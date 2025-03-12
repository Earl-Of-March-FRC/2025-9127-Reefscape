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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoRoutines.Routines.TimedRoutines.ExitZoneCommand;
import frc.robot.commands.AutoRoutines.Routines.ToReefScore.Score;
import frc.robot.commands.AutoRoutines.Routines.ToReefScore.ScoreandStation;
import frc.robot.commands.DriveFieldOriented;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ReverseCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootL1Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeSubsystem;


public class RobotContainer {

  private final SendableChooser<Command> autoChooser;
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final XboxController driveController = new XboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final Elevator elevator = new Elevator();
  private ElevatorPID[] elevatorCommands;
  private int elevatorPositionIndex;

  private final IntakeSubsystem intakeSub = new IntakeSubsystem();

  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
public RobotContainer() {

    autoChooser = AutoBuilder.buildAutoChooser();

    // elevatorPositionIndex = 0;
    // elevatorCommands = new ElevatorPID[]{
    //   new ElevatorPID(elevator, Constants.ElevatorConstants.INTAKE_POSITION),
    //   new ElevatorPID(elevator, Constants.ElevatorConstants.L1_POSITION),
    //   new ElevatorPID(elevator, Constants.ElevatorConstants.L2_POSITION),
    //   new ElevatorPID(elevator, Constants.ElevatorConstants.L3_POSITION),
    //   new ElevatorPID(elevator, Constants.ElevatorConstants.L4_POSITION)
    // };

    // Configure the trigger bindings
    drivetrain.setDefaultCommand(new DriveFieldOriented(
      drivetrain,
      () -> (driveController.getLeftX()), //X translation
      () -> -(driveController.getLeftY()), //Y translation
      () -> (driveController.getRightX()) //Z rotation
      ));
    configureBindings();

    //Go to Reef commands
    autoChooser.addOption("GO TO REEF FROM RIGHT", new PathPlannerAuto("To reef from right"));
    autoChooser.addOption("GO TO REEF FROM LEFT", new PathPlannerAuto("To reef from left"));
    autoChooser.addOption("GO TO REEF FROM CENTRE", new PathPlannerAuto("To reef from centre"));

    /////Score Commands/////

    //Score L1
    //autoChooser.addOption("SCORE L1 FROM RIGHT", new Score(intakeSub, "To reef from right", elevator, ElevatorConstants.L1_POSITION));
    //autoChooser.addOption("SCORE L1 FROM LEFT", new Score(intakeSub, "To reef from left", elevator, ElevatorConstants.L1_POSITION));
    //autoChooser.addOption("SCORE L1 FROM CENTRE", new Score(intakeSub, "To reef from centre", elevator, ElevatorConstants.L1_POSITION));

    //Score L2
    autoChooser.addOption("SCORE L2 FROM RIGHT", new Score(intakeSub, "To reef from right", elevator, ElevatorConstants.L2_POSITION));
    autoChooser.addOption("SCORE L2 FROM LEFT", new Score(intakeSub, "To reef from left", elevator, ElevatorConstants.L2_POSITION));
    autoChooser.addOption("SCORE L2 FROM CENTRE", new Score(intakeSub, "To reef from centre", elevator, ElevatorConstants.L2_POSITION));

    //Score L3
    autoChooser.addOption("SCORE L3 FROM RIGHT", new Score(intakeSub, "To reef from right", elevator, ElevatorConstants.L3_POSITION));
    autoChooser.addOption("SCORE L3 FROM LEFT", new Score(intakeSub, "To reef from left", elevator, ElevatorConstants.L3_POSITION));
    autoChooser.addOption("SCORE L3 FROM CENTRE", new Score(intakeSub, "To reef from centre", elevator, ElevatorConstants.L3_POSITION));
    

    // //Score L4
    // autoChooser.addOption("SCORE L4 FROM RIGHT", new Score(intakeSub, "To reef from right", elevator, ElevatorConstants.L4_POSITION));
    // autoChooser.addOption("SCORE L4 FROM LEFT", new Score(intakeSub, "To reef from left", elevator, ElevatorConstants.L4_POSITION));
    // autoChooser.addOption("SCORE L4 FROM CENTRE", new Score(intakeSub, "To reef from centre", elevator, ElevatorConstants.L4_POSITION));
    
    //Score L2 
    autoChooser.addOption("SCORE L2 FROM RIGHT AND INTAKE", new ScoreandStation(intakeSub, "To reef from right", elevator, ElevatorConstants.L2_POSITION, "Intake from right"));
    autoChooser.addOption("SCORE L2 FROM LEFT AND INTAKE", new ScoreandStation(intakeSub, "To reef from left", elevator, ElevatorConstants.L2_POSITION, "Intake from left"));
    autoChooser.addOption("SCORE L2 FROM CENTRE AND INTAKE", new ScoreandStation(intakeSub, "To reef from centre", elevator, ElevatorConstants.L2_POSITION,"Intake from centre"));

    //Score L3
    autoChooser.addOption("SCORE L3 FROM RIGHT AND INTAKE", new ScoreandStation(intakeSub, "To reef from right", elevator, ElevatorConstants.L3_POSITION, "Intake from right"));
    autoChooser.addOption("SCORE L3 FROM LEFT AND INTAKE", new ScoreandStation(intakeSub, "To reef from left", elevator, ElevatorConstants.L3_POSITION, "Intake from left"));
    autoChooser.addOption("SCORE L3 FROM CENTRE AND INTAKE", new ScoreandStation(intakeSub, "To reef from centre", elevator, ElevatorConstants.L3_POSITION, "Intake from centre"));

    //Exit Zone timed
    //autoChooser.addOption("EXIT ZONE TIMED", new ExitZoneCommand(drivetrain, 0.5, 1));

    autoChooser.setDefaultOption("EXIT ZONE TIMED", new ExitZoneCommand(drivetrain, 0.5, 1));
    
    SmartDashboard.putData("Autonomous Routine", autoChooser);

    //elevator.setDefaultCommand(new ManualElevator(elevator, ()-> -m_operatorController.getRightTriggerAxis() + m_operatorController.getLeftTriggerAxis()));
  }
    
  

  private void configureBindings() {
    // Configure your button bindings here
  
    //Reset the gyro angle to 0 when A is pressed on the driver controller
    new Trigger(driveController::getAButtonPressed).onTrue(Commands.runOnce(() -> drivetrain.resetGyro(), drivetrain));
    
    //Toggle the drive mode (field or robot oriented) when B is pressed on the driver controller
    new Trigger(driveController::getBButtonPressed).onTrue(Commands.runOnce(() -> drivetrain.changeDriveMode(), drivetrain));

    //automatically intake with beam break sensor using button a
    operatorController.a().whileTrue(new IntakeCommand(intakeSub));

    operatorController.b().whileTrue(new ShootCommand(intakeSub, () -> 0.4));
    
    operatorController.y().whileTrue(new ShootL1Command(intakeSub));

    //reverse direction for intake with right trigger
    new Trigger(() -> Math.abs(operatorController.getRightY()) > 0.1)
        .whileTrue(new ReverseCommand(
            intakeSub,
            () -> operatorController.getRightY()
        ));

    // operatorController.leftBumper().onTrue(new InstantCommand(()->{
    //   elevatorPositionIndex = (elevatorPositionIndex + 1) % elevatorCommands.length;
    //   elevatorCommands[elevatorPositionIndex].schedule();
    // }, elevator).until(() -> operatorController.getRightTriggerAxis() > 0.1 || operatorController.getLeftTriggerAxis() > 0.1 )
    // );
    
    // operatorController.rightBumper().onTrue(new InstantCommand(()->{
    //   elevatorPositionIndex = (elevatorPositionIndex - 1 + elevatorCommands.length) % elevatorCommands.length;
    //   elevatorCommands[elevatorPositionIndex].schedule();
    // }, elevator).until(() -> operatorController.getRightTriggerAxis() > 0.1 || operatorController.getLeftTriggerAxis() > 0.1 )
    // );

    operatorController.povDown().onTrue(new InstantCommand(() -> new ElevatorPID(elevator, ElevatorConstants.L2_POSITION).schedule(), elevator));
    operatorController.povUp().onTrue(new InstantCommand(() -> new ElevatorPID(elevator, ElevatorConstants.L4_POSITION).schedule(), elevator));
    operatorController.povLeft().onTrue(new InstantCommand(() -> new ElevatorPID(elevator, ElevatorConstants.L1_POSITION).schedule(), elevator));
    operatorController.povRight().onTrue(new InstantCommand(() -> new ElevatorPID(elevator, ElevatorConstants.L3_POSITION).schedule(), elevator));
    
    operatorController.x().onTrue(new InstantCommand(() -> new ElevatorPID(elevator, ElevatorConstants.INTAKE_POSITION).schedule(), elevator));

    
    operatorController.rightBumper().onTrue(new InstantCommand(() -> new ElevatorPID(elevator, elevator.getPosition()+ElevatorConstants.MANUAL_OFFSET).schedule(), elevator));
    operatorController.leftBumper().onTrue(new InstantCommand(() -> new ElevatorPID(elevator, elevator.getPosition()-ElevatorConstants.MANUAL_OFFSET).schedule(), elevator));
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}