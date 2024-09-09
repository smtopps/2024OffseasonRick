// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootCommands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;

public class RevShooter extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Shooter shooter;

  private double distanceToTarget;
  private int targetTag;
  private Pose2d targetPose;
  private boolean previousState;
  private boolean currentState;
  private final DoubleSupplier maxSpeed;
  /** Creates a new RevShooter. */
  public RevShooter(CommandSwerveDrivetrain drivetrain, Shooter shooter, DoubleSupplier maxSpeed) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.maxSpeed = maxSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    previousState = false;
    currentState = false;
    targetTag = DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?7:4;
    targetPose = ShooterConstants.aprilTags.getTagPose(targetTag).get().toPose2d();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;
    distanceToTarget = PhotonUtils.getDistanceToPose(currentPose, targetPose);
    if(distanceToTarget < Units.inchesToMeters(160) && maxSpeed.getAsDouble() == TunerConstants.kSpeedAt12VoltsMps) {
      currentState = true;
    }else{
      currentState = false;
    }

    if(currentState != previousState && currentState == true) {
      shooter.setShooterSpeeds(ShooterConstants.ampHandoffRPS, 0.0);
      previousState = true;
    }else if(currentState != previousState && currentState == false) {
      shooter.stopShooter();
      previousState = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    previousState = false;
    currentState = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
