package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PneumaticsAlgaeRemove;

public class AlgaeRemove extends Command {
    private final PneumaticsAlgaeRemove pneumaticsSubsystem;

    public AlgaeRemove(PneumaticsAlgaeRemove pneumaticsSubsystem) {
        this.pneumaticsSubsystem = pneumaticsSubsystem;
        addRequirements(pneumaticsSubsystem);
    }

    @Override
    public void initialize() {
        pneumaticsSubsystem.extend();
    }

    @Override
    public void end(boolean interrupted) {
        pneumaticsSubsystem.retract();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs while button is held
    }
}
