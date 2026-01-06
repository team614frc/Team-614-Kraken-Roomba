// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private static final int kPort = 9;
  private static final int kLength = 7;
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    led = new AddressableLED(kPort);
    buffer = new AddressableLEDBuffer(kLength);
    led.setLength(kLength);
    led.start();
  }

  @Override
  public void periodic() {
    if (!DriverStation.isDSAttached()) {
      // Create custom RGB color (orange-ish), values from 0.0 to 1.0
      Color customOrange = new Color(255, 50, 0);

      // Apply a blinking pattern with 1-second interval
      LEDPattern blinkPattern = LEDPattern.solid(customOrange).blink(Seconds.of(0.02));
      blinkPattern.applyTo(buffer);
    } else if (DriverStation.isAutonomousEnabled()) {
      // LEDPattern m_scrollingRainbow =
      //     LEDPattern.rainbow(255, 255)
      //         .scrollAtAbsoluteSpeed(MetersPerSecond.of(2), Meters.of(1 / 6));
      // m_scrollingRainbow.applyTo(buffer);
      // LEDPattern base =
      //     LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlue, Color.kAqua);
      // LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Seconds).of(150));
      // pattern.applyTo(buffer);
    } else if (DriverStation.isTeleopEnabled()) {
      LEDPattern base =
          LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kBlue, Color.kAqua);
      LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Seconds).of(150));
      pattern.applyTo(buffer);
    } else if (DriverStation.isDSAttached()) {
      LEDPattern ready = LEDPattern.solid(Color.kGreen);
      ready.applyTo(buffer);
    }

    // 🔁 FIX: Swap R and G channels before sending to LED strip
    for (int i = 0; i < buffer.getLength(); i++) {
      int r = buffer.getRed(i);
      int g = buffer.getGreen(i);
      int b = buffer.getBlue(i);
      buffer.setRGB(i, g, r, b); // Swap R and G
    }

    led.setData(buffer);
  }
}
