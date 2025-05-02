// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DrivebaseConstants {
    public static final double MAX_SPEED = Units.feetToMeters(14.5);

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static final class LEDConstants {
    public static final int PORT = 6;
    public static final int LEGNTH = 10;
    public static final Color BASIC_COLOR = new Color(101, 255 , 1); //red is green, green is red, blue is blue  
    public static final Color ALIGNMENT_COLOR = new Color(248, 31 , 9); //red is green, green is red, blue is blue  
    public static final Distance SPACING = Meters.of(1 / 120.0);
    public static final LinearVelocity SCROLL_SPEED = MetersPerSecond.of(.1);
    public static final Time BREATHE_TIME = Seconds.of(2);
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }
}
