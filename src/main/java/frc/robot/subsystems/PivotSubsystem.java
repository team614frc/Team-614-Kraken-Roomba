// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants.PivotSetpoint;

public class PivotSubsystem extends SubsystemBase {
  private final SparkFlex pivotMotor =
      new SparkFlex(PivotConstants.PIVOT_MOTOR, MotorType.kBrushless);
  private AbsoluteEncoder pivotEncoder = pivotMotor.getAbsoluteEncoder();
  private PivotSetpoint pivotSetpoint = PivotSetpoint.PIVOT_IDLE;

  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          PivotConstants.PIVOT_kP,
          PivotConstants.PIVOT_kI,
          PivotConstants.PIVOT_kD,
          new TrapezoidProfile.Constraints(
              Units.rotationsToRadians(PivotConstants.PIVOT_MAX_VELOCITY),
              Units.rotationsToRadians(PivotConstants.PIVOT_MAX_ACCELERATION)));

  private final ArmFeedforward feedforward =
      new ArmFeedforward(
        PivotConstants.PIVOT_kS,
        PivotConstants.PIVOT_kG,
        PivotConstants.PIVOT_kV,
        PivotConstants.PIVOT_kA);

  public PivotSubsystem() {
    pivotMotor.configure(
        Configs.PivotConfig.PIVOT_CONFIG,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public boolean atSetpoint() {
    return Math.abs(pivotEncoder.getPosition() - pivotSetpoint.value.in(Rotations))
        <= PivotConstants.PIVOT_TOLERANCE;
  }

  private double getPivotAngleRadians() {
    return Units.rotationsToRadians(pivotEncoder.getPosition());
  }

  private void pivotPIDControl() {
    double armFeedforwardVoltage =
        feedforward.calculate(
            pid.getSetpoint().position
                - Units.rotationsToRadians(PivotConstants.PIVOT_FEEDFORWARD_OFFSET),
            pid.getSetpoint().velocity);

    double ArmPidOutput =
        pid.calculate(getPivotAngleRadians(), pivotSetpoint.value.in(Radians));

    pivotMotor.setVoltage(ArmPidOutput + armFeedforwardVoltage);
  }

  @Override
  public void periodic() {
    pivotPIDControl();
    SmartDashboard.putData(pid);
    SmartDashboard.putNumber("Pivot Goal", pid.getGoal().position);
    SmartDashboard.putNumber("Pivot Position", pivotEncoder.getPosition());
  }

  public Command pivotDown() {
    return this.runOnce(
        () -> {
          pivotSetpoint = PivotSetpoint.PIVOT_DOWN;
        });
  }

  public Command pivotIdle() {
    return this.runOnce(
        () -> {
          pivotSetpoint = PivotSetpoint.PIVOT_IDLE;
        });
  }

  public Command pivotUp() {
    return this.runOnce(
        () -> {
          pivotSetpoint = PivotSetpoint.PIVOT_UP;
        });
  }
}
