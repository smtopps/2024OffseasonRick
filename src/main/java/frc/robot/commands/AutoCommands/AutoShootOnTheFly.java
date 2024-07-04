// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AutoShootOnTheFly extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Intake intake;
  private final Shooter shooter;
  private final double speed = ShooterConstants.shootingRPS;
  private final double spin = ShooterConstants.spinFactor;
  private int targetTag;
  private Pose2d targetPose;
  private double distanceToTarget;

  private boolean finished = false;
  private boolean timeStampLock = true;
  private double shootTime = 0;
  /** Creates a new AutoShootOnTheFly. */
  public AutoShootOnTheFly(CommandSwerveDrivetrain drivetrain, Shooter shooter, Intake intake) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterSpeeds(speed, spin);
    intake.setIntakePosition(IntakeConstants.shootPosition);
    targetTag = DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?7:4;
    targetPose = ShooterConstants.aprilTags.getTagPose(targetTag).get().toPose2d();
    timeStampLock = true;
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;
    distanceToTarget = PhotonUtils.getDistanceToPose(currentPose, targetPose);
    shooter.log("Distance To Target", distanceToTarget);

    if(intake.isIntakeAtPosition(IntakeConstants.shootPosition) && shooter.isShooterAtSpeed(ShooterConstants.shootingRPS, ShooterConstants.spinFactor) && distanceToTarget < 120) {
      intake.setRollerSpeed(IntakeConstants.shootSpeed);
      if(timeStampLock){
        shootTime = Timer.getFPGATimestamp();
        timeStampLock = false;
      }

      if(!timeStampLock && Timer.getFPGATimestamp() - shootTime > 0.2){
        finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setRollerSpeed(IntakeConstants.stallSpeed);
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
