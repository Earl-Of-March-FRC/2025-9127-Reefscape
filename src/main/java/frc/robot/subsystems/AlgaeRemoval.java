// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeRemovalConstants;

public class AlgaeRemoval extends SubsystemBase {
  private final Servo servo;

  /** Creates a new AlageRemoval. */
  public AlgaeRemoval() {
    servo = new Servo(AlgaeRemovalConstants.SERVO_PORT);
    //servo.setBoundsMicroseconds(2400, 0, 1500, 0, 600);
    downPosition();
  }

  public double getPosition() {
    return servo.getPosition();
  }

  public void setAngle(double angle) {
    servo.setAngle(angle);
  }

  public void setPosition(double position) {
    servo.setPosition(position);
  }

  public void upPosition() {
    servo.setPosition(AlgaeRemovalConstants.UP_POSITION);
  }

  public void downPosition() {
    servo.setPosition(AlgaeRemovalConstants.DOWN_POSITION);
  }

  public void togglePosition() {
    if (servo.getPosition() == AlgaeRemovalConstants.UP_POSITION) {
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
