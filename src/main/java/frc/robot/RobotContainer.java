package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlgaeRemove;
import frc.robot.subsystems.PneumaticsAlgaeRemove;

public class RobotContainer {
    private final XboxController controller = new XboxController(0);
    private final PneumaticsAlgaeRemove pneumaticsSubsystem = new PneumaticsAlgaeRemove();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        new JoystickButton(controller, XboxController.Button.kX.value)
            .whileTrue(new AlgaeRemove(pneumaticsSubsystem));
    }

    public Command getAutonomousCommand() {
      return null;
    }
}
