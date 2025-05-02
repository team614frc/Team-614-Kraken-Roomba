// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;

  public LEDSubsystem() {
    led = new AddressableLED(LEDConstants.PORT);
    buffer = new AddressableLEDBuffer(LEDConstants.LEGNTH);
    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
    setBasicPattern();
    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    setDefaultCommand(setBasicPattern());
  }

  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display
    led.setData(buffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(buffer));
  }

  public Command setBasicPattern() {
    LEDPattern pattern;
    pattern = LEDPattern.solid(Constants.LEDConstants.BASIC_COLOR);
    return runPattern(pattern);
  }

  public Command setIntakePattern() {
    LEDPattern scrollingRainbow =
        LEDPattern.rainbow(255, 255)
            .scrollAtAbsoluteSpeed(
                Constants.LEDConstants.SCROLL_SPEED, Constants.LEDConstants.SPACING);
    return runPattern(scrollingRainbow);
  }

  public Command setAlignmentPattern() {
    LEDPattern pattern =
        LEDPattern.solid(Constants.LEDConstants.ALIGNMENT_COLOR)
            .breathe(Constants.LEDConstants.BREATHE_TIME);
    return runPattern(pattern);
  }
}
