
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;

public class IntakeGroundAlignDrive extends Command {
  private final Intake intake;
  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier triggerTranslation;
  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final DoubleSupplier rotation;

  private final double triggerThreshold = 0.5;

  private PIDController yawPIDController = new PIDController(0.1, 0.0, 0.0);

  private SwerveRequest.FieldCentric fieldRequest;
  private SwerveRequest.RobotCentric robotRequest = new SwerveRequest.RobotCentric();

  private DoubleSupplier maxSpeed;

  /** Creates a new Intake. */
  public IntakeGroundAlignDrive(Intake intake, CommandSwerveDrivetrain drivetrain, DoubleSupplier triggerTranslation, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, SwerveRequest.FieldCentric fieldRequest, DoubleSupplier maxSpeed) {
    this.intake = intake;
    this.drivetrain = drivetrain;
    this.triggerTranslation = triggerTranslation;
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;
    this.fieldRequest = fieldRequest;
    this.maxSpeed = maxSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakePosition(IntakeConstants.floorPosition);
    intake.setRollerSpeed(IntakeConstants.floorSpeed);
    LimelightHelpers.setPipelineIndex("limelight-intake", 0);
    yawPIDController.setSetpoint(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double topSpeed = 2.0;
    if(maxSpeed.getAsDouble() == TunerConstants.kSpeedAt12VoltsMps) {
      topSpeed = 2.0;
    }else{
      topSpeed = maxSpeed.getAsDouble();
    }
    if(LimelightHelpers.getTV("limelight-intake")){
      double yawSpeed = yawPIDController.calculate(LimelightHelpers.getTX("limelight-intake"));
      double xSpeed = MathUtil.inverseInterpolate(triggerThreshold, 1.0, triggerTranslation.getAsDouble());
      //xSpeed = Math.pow(xSpeed, 3.0);
      xSpeed = MathUtil.interpolate(0.25, topSpeed, xSpeed);
      if(triggerTranslation.getAsDouble() >= triggerThreshold && intake.isIntakeAtPosition(IntakeConstants.floorPosition)) {
        drivetrain.setControl(robotRequest.withRotationalRate(yawSpeed).withVelocityX(xSpeed).withVelocityY(translationY.getAsDouble()));
      }else{
        drivetrain.setControl(fieldRequest.withRotationalRate(yawSpeed).withVelocityX(translationX.getAsDouble()).withVelocityY(translationY.getAsDouble()));
      }
    }else{
      drivetrain.setControl(fieldRequest.withRotationalRate(rotation.getAsDouble()).withVelocityX(translationX.getAsDouble()).withVelocityY(translationY.getAsDouble()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setRollerSpeed(IntakeConstants.stallSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
