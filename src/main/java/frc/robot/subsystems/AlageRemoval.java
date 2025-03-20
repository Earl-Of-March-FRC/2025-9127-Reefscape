// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlageRemovalConstants;

public class AlageRemoval extends SubsystemBase {
  private final Servo servo;

  /** Creates a new AlageRemoval. */
  public AlageRemoval() {
    servo = new Servo(AlageRemovalConstants.SERVO_PORT);
  }

  public void setAngle(double angle) {
    servo.setAngle(angle);
  }

  public void setPosition(double position) {
    servo.setPosition(position);
  }

  public void upPosition() {
    servo.setPosition(AlageRemovalConstants.UP_POSITION);
  }

  public void downPosition() {
    servo.setPosition(AlageRemovalConstants.DOWN_POSITION);
  }

  public void togglePosition() {
    if (servo.getPosition() == AlageRemovalConstants.UP_POSITION) {
      downPosition();
    } else {
      upPosition();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
