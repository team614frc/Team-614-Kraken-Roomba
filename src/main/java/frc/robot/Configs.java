package frc.robot;

import static edu.wpi.first.units.Units.Amp;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public final class Configs {

  public static final class IntakeConfig {
    public static final SparkFlexConfig INTAKE_CONFIG = new SparkFlexConfig();

    static {
      INTAKE_CONFIG
          .idleMode(IdleMode.kBrake)
          .inverted(true)
          .smartCurrentLimit((int) Constants.IntakeConstants.INTAKE_CURRENT_LIMIT.in(Amp));
    }
  }

  public static final class PivotConfig {
    public static final SparkFlexConfig PIVOT_CONFIG = new SparkFlexConfig();

    static {
      PIVOT_CONFIG
          .idleMode(IdleMode.kBrake)
          .inverted(true)
          .smartCurrentLimit((int) Constants.IntakeConstants.INTAKE_PIVOT_CURRENT_LIMIT.in(Amp));
    }
  }
}
