// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;

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
  }

  public static class OperatorConstants {
    public static final double DEADBAND = 0.1;
  }

  public static final class IntakeConstants {
    public static final int INTAKE_MOTOR = 5;
    public static final Current INTAKE_CURRENT_LIMIT = Amp.of(80);
    public static final double LOW_SPEED = -0.2;
    public static final double MID_SPEED = -0.3;
    public static final double HIGH_SPEED = -0.5;
    public static final double INTAKE_SPEED = 0.25;
    public static final double INTAKE_REST_SPEED = 0;
    public static final double OUTTAKE_REST_SPEED = 0;
  }

  public static class PivotConstants {
    public static final int PIVOT_MOTOR = 6;
    public static final Current PIVOT_CURRENT_LIMIT = Amp.of(80);
    public static final double PIVOT_MAX_VELOCITY = 2;
    public static final double PIVOT_MAX_ACCELERATION = 2;
    public static final double PIVOT_FEEDFORWARD_OFFSET = 0.32;
    public static final double PIVOT_TOLERANCE = 0.75;
    public static final double PIVOT_kP = 0.1;
    public static final double PIVOT_kI = 0;
    public static final double PIVOT_kD = 0;
    public static final double PIVOT_kS = 0;
    public static final double PIVOT_kG = 0;
    public static final double PIVOT_kV = 0;
    public static final double PIVOT_kA = 0;

    public enum PivotSetpoint {
      PIVOT_UP(0.03),
      PIVOT_DOWN(0.465),
      PIVOT_IDLE(0.49);

      public final Angle value;

      PivotSetpoint(double value) {
        this.value = Rotations.of(value);
      }
    }
  }
}
