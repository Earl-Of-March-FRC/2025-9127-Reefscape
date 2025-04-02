// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDsubsystem extends SubsystemBase {

  private final NetworkTable m_limelightTable1;
  private final NetworkTable m_limelightTable2;
  private final BooleanSupplier OFF, ON, BLINK;

  /** Creates a new LEDsubsystem. */
  public LEDsubsystem(NetworkTable limelight1, NetworkTable limelight2, BooleanSupplier OFF, BooleanSupplier ON, BooleanSupplier BLINK) {
    //Use limelight LEDS
    m_limelightTable1 = limelight1;
    m_limelightTable2 = limelight2;

    this.OFF = OFF;
    this.ON = ON;
    this.BLINK = BLINK;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (OFF.getAsBoolean()) {
      setLedMode(LedMode.OFF);
    }
    else if (ON.getAsBoolean()) {
      setLedMode(LedMode.ON);
    }
    else if (BLINK.getAsBoolean()) {
      setLedMode(LedMode.BLINK);
    }
  }
  
  public void setLedMode(LedMode mode) {
    m_limelightTable1.getEntry("ledMode").setNumber(mode.getValue());
    m_limelightTable2.getEntry("ledMode").setNumber(mode.getValue());
  }

  public enum LedMode {
    PIPELINE(0),  // Use LED mode from pipeline
    OFF(1),       // Force LEDs off
    BLINK(2),     // Force LEDs to blink
    ON(3);        // Force LEDs on
    
    private final int value;
    
    LedMode(int value) {
        this.value = value;
    }
    
    public int getValue() {
        return value;
    }
  }
}

