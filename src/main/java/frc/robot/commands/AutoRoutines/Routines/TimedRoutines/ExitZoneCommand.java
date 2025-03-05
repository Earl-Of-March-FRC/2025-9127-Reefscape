// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoRoutines.Routines.TimedRoutines;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;


public class ExitZoneCommand extends Command {
    private final Drivetrain drivetrain;
    private final double speed;
    private final double duration;
    private long startTime;

    public ExitZoneCommand(Drivetrain drivetrain, double speed, double duration) {
        this.drivetrain = drivetrain;
        this.speed = speed;
        this.duration = duration * 1000; // Convert to milliseconds
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        //TODO confirm directions
        drivetrain.drive(new ChassisSpeeds(speed, 0, 0));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return System.currentTimeMillis() - startTime >= duration;
    }
}
