// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import monologue.Annotations.Log;
import monologue.LogLevel;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged{
  private final TalonFX rotationMotor = new TalonFX(IntakeConstants.rotationMotorID);
  private final CANSparkFlex rollerMotor = new CANSparkFlex(IntakeConstants.rollerMotorID, MotorType.kBrushless);

  private final MotionMagicVoltage magicRequest = new MotionMagicVoltage(0);

  @Log.NT(level = LogLevel.DEFAULT) Pose3d intakePose;

  /** Creates a new Intake. */
  public Intake() {
    configureRotationMotor();
    configureRollerMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakePose = new Pose3d(0.32004, 0, 0.280616, new Rotation3d(0, -Units.rotationsToRadians(rotationMotor.getPosition().getValueAsDouble()), 0));
  }
  
  private void configureRotationMotor() {
    rotationMotor.getConfigurator().apply(new TalonFXConfiguration());
    var talonfxConfigs = new TalonFXConfiguration();
    talonfxConfigs.Slot0 = IntakeConstants.rotationSlot0Configs;
    talonfxConfigs.CurrentLimits = IntakeConstants.rotationCurrentLimits;
    talonfxConfigs.Voltage = IntakeConstants.rotationVoltageConfigs;
    talonfxConfigs.Feedback = IntakeConstants.rotationFeedbackConfigs;
    talonfxConfigs.MotionMagic = IntakeConstants.rotationMotionMagicConfigs;
    talonfxConfigs.SoftwareLimitSwitch = IntakeConstants.rotationSoftwareLimitSwitchConfigs;
    talonfxConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rotationMotor.getConfigurator().apply(talonfxConfigs);
    rotationMotor.setPosition(IntakeConstants.stowedPosition);
    setIntakePosition(IntakeConstants.stowedPosition);
  }

  private void configureRollerMotor() {
    rollerMotor.restoreFactoryDefaults();
    rollerMotor.setIdleMode(IdleMode.kCoast);
    rollerMotor.setSmartCurrentLimit(IntakeConstants.rollerMotorCurrentLimit); //55
    rollerMotor.setInverted(true);
    rollerMotor.set(IntakeConstants.stallSpeed);
  }

  public void setIntakeRollerCurrentLimit(int current) {
    rollerMotor.setSmartCurrentLimit(current);
  }

  public void setIntakePosition(double position) {
    rotationMotor.setControl(magicRequest.withPosition(position).withSlot(0));
  }

  public void setRollerSpeed(double speed) {
    rollerMotor.set(speed);
  }

  public void stopRotation() {
    rotationMotor.stopMotor();
  }

  public void stopRoller() {
    rollerMotor.stopMotor();
  }

  public boolean isIntakeAtPosition(double position) {
    double error = Math.abs(position - rotationMotor.getPosition().getValue());
    if(error < IntakeConstants.positionError) {
      return true;
    }else{
      return false;
    }
  }
}
