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
    //HS 3-22 HD servo specs
    servo.setBoundsMicroseconds(2100, 5, 1500, 5, 900);
    upPosition();
  }

  public double getPosition() {
    return servo.getPosition();
  }

  public void setAngle(double angle) {
    servo.setAngle(angle);
  }

  public void setPosition(double position) {
    servo.set(position);
  }

  public void upPosition() {
    // servo.set(AlgaeRemovalConstants.UP_POSITION);
    servo.setAngle(180);
  }

  public void downPosition() {
    // servo.set(AlgaeRemovalConstants.DOWN_POSITION);
    servo.setAngle(0.2*180.0);
  }

  public void togglePosition() {
    if (servo.getPosition() == AlgaeRemovalConstants.UP_POSITION) {
      System.out.println("DOWN POS");
      downPosition();
    } else {
      System.out.println("UP POS");
      upPosition();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
