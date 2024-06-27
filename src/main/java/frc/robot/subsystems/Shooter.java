// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import monologue.Annotations.Log;
import monologue.LogLevel;
import monologue.Logged;

public class Shooter extends SubsystemBase implements Logged {
  private TalonFX shootLeft = new TalonFX(ShooterConstants.leftShooterMotorID);
  private TalonFX shootRight = new TalonFX(ShooterConstants.rightShooterMotorID);

  final VelocityVoltage voltageRequest = new VelocityVoltage(0);

  @Log.File(level = LogLevel.DEFAULT) double leftShooterSpeed;
  @Log.File(level = LogLevel.DEFAULT) double rightShooterSpeed;
  @Log.File(level = LogLevel.DEFAULT) double leftShooterSetpoint;
  @Log.File(level = LogLevel.DEFAULT) double rightShooterSetpoint;

  /** Creates a new Shooter. */
  public Shooter() {
    configShooterMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftShooterSpeed = shootLeft.getVelocity().getValueAsDouble();
    rightShooterSpeed = shootRight.getVelocity().getValueAsDouble();
  }

   private void configShooterMotors() {
    shootLeft.getConfigurator().apply(new TalonFXConfiguration());
    shootRight.getConfigurator().apply(new TalonFXConfiguration());
    var talonfxConfigs = new TalonFXConfiguration();
    talonfxConfigs.Slot0 = ShooterConstants.shooterSlot0Configs;
    talonfxConfigs.CurrentLimits = ShooterConstants.shooterCurrentLimits;
    talonfxConfigs.Voltage = ShooterConstants.shooterVoltageConfigs;
    talonfxConfigs.Feedback = ShooterConstants.shooterFeedbackConfigs;
    talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shootLeft.getConfigurator().apply(talonfxConfigs);
    shootRight.getConfigurator().apply(talonfxConfigs);
    shootRight.setInverted(true);
  }

  public void setShooterSpeeds(double RPS, double spinFactor) {
    shootLeft.setControl(voltageRequest.withVelocity(RPS*(1-spinFactor)));
    shootRight.setControl(voltageRequest.withVelocity(RPS*(1+spinFactor)));
  }

  public void stopShooter() {
    shootLeft.stopMotor();
    shootRight.stopMotor();
  }
  
  public boolean isShooterAtSpeed(double RPS, double spinFactor) {
    double shootLeftError = Math.abs(shootLeft.getVelocity().getValue()-(RPS*(1-spinFactor)));
    double shootRightError = Math.abs(shootRight.getVelocity().getValue()-(RPS*(1+spinFactor)));
    if(shootLeftError<ShooterConstants.speedThreshold && shootRightError<ShooterConstants.speedThreshold) {
      return true;
    }else{
      return false;
    }
  }
}
