package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PneumaticsAlgaeRemove extends SubsystemBase {
    private final DoubleSolenoid solenoid;

    public PneumaticsAlgaeRemove() {
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1); // Ports 0 and 1
    }

    public void extend() {
        solenoid.set(kForward);
    }

    public void retract() {
        solenoid.set(kReverse);
    }
}
