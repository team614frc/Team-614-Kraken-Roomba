// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex intakeMotor =
      new SparkFlex(IntakeConstants.INTAKE_MOTOR, MotorType.kBrushless);

  public IntakeSubsystem() {
    intakeMotor.configure(
        Configs.IntakeConfig.INTAKE_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {}

  public void set(double speed) {
    intakeMotor.set(speed);
  }

  public Command intake() {
    return this.runEnd(
        () -> {
          set(IntakeConstants.INTAKE_SPEED);
        },
        () -> {
          set(IntakeConstants.INTAKE_REST_SPEED);
        });
  }

  public Command scoreLow() {
    return this.runEnd(
        () -> {
          set(IntakeConstants.LOW_SPEED);
        },
        () -> {
          set(IntakeConstants.OUTTAKE_REST_SPEED);
        });
  }

  public Command scoreMid() {
    return this.runEnd(
        () -> {
          set(IntakeConstants.MID_SPEED);
        },
        () -> {
          set(IntakeConstants.OUTTAKE_REST_SPEED);
        });
  }

  public Command scoreHigh() {
    return this.runEnd(
        () -> {
          set(IntakeConstants.HIGH_SPEED);
        },
        () -> {
          set(IntakeConstants.OUTTAKE_REST_SPEED);
        });
  }

  public Command stopIntake() {
    return this.runOnce(() -> set(IntakeConstants.INTAKE_REST_SPEED));
  }
}
