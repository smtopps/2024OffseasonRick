// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;

public class AutoShootPose extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Shooter shooter;
  private final Intake intake;

  private final PIDController yawController = new PIDController(0.1, 0, 0);
  private final PIDController distanceController = new PIDController(3.5, 0, 0.02);

  private Rotation2d rotationToTarget;
  private double distanceToTarget;

  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

  private int targetTag;
  private Pose2d targetPose;

  private boolean finished = false;
  private boolean timeStampLock = true;
  private double shootTime = 0;
  /** Creates a new ShootPose. */
  public AutoShootPose(CommandSwerveDrivetrain drivetrain, Shooter shooter, Intake intake) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterSpeeds(ShooterConstants.shootingRPS, ShooterConstants.spinFactor);
    intake.setIntakePosition(IntakeConstants.shootPosition);
    targetTag = DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?7:4;
    yawController.setSetpoint(180.0);
    yawController.setTolerance(2.0);
    yawController.enableContinuousInput(-180, 180);
    distanceController.setSetpoint(2.9); //3.0
    distanceController.setTolerance(Units.inchesToMeters(2.0));
    targetPose = ShooterConstants.aprilTags.getTagPose(targetTag).get().toPose2d();
    timeStampLock = true;
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;
    rotationToTarget = PhotonUtils.getYawToPose(currentPose, targetPose);
    distanceToTarget = PhotonUtils.getDistanceToPose(currentPose, targetPose);
    shooter.log("Yaw To Target", rotationToTarget.getDegrees());
    shooter.log("Distance To Target", distanceToTarget);

    double yawSpeed = yawController.calculate(-rotationToTarget.getDegrees());
    yawSpeed = MathUtil.clamp(yawSpeed, -3.8, 3.8);
    double distanceSpeed;
    if(Math.abs(yawController.getPositionError()) < 10.0) {
      distanceSpeed = distanceController.calculate(distanceToTarget); 
      distanceSpeed = MathUtil.clamp(distanceSpeed, -4.0, 4.0);
    }else{
      distanceSpeed = 0.0;
    }

    drivetrain.setControl(drive.withRotationalRate(yawSpeed).withVelocityX(distanceSpeed).withVelocityY(0));

    shooter.log("isYawAtPosition", yawController.atSetpoint());
    shooter.log("isDistanceAtPosition", distanceController.atSetpoint());
    intake.log("isIntakeAtPosition", intake.isIntakeAtPosition(IntakeConstants.shootPosition));
    shooter.log("isShooterAtSpeed", shooter.isShooterAtSpeed(ShooterConstants.shootingRPS, ShooterConstants.spinFactor));
    
    if(yawController.atSetpoint() && distanceController.atSetpoint() && intake.isIntakeAtPosition(IntakeConstants.shootPosition) && shooter.isShooterAtSpeed(ShooterConstants.shootingRPS, ShooterConstants.spinFactor)) {
      intake.setRollerSpeed(IntakeConstants.shootSpeed);
      if(timeStampLock){
        shootTime = Timer.getFPGATimestamp();
        timeStampLock = false;
      }

      if(!timeStampLock && Timer.getFPGATimestamp() - shootTime > 0.2){
        finished = true;
      }
    };
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
