// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootCommands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonUtils;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;

public class ShootToZone extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Shooter shooter;
  private final Intake intake;

  private final PIDController yawController = new PIDController(0.1, 0, 0);

  private Rotation2d rotationToTarget;
  private double distanceToTarget;

  private final DoubleSupplier TranslationX;
  private final DoubleSupplier TranslationY;
  private double MaxSpeed;
  private double MaxAngularRate;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.02).withRotationalDeadband(MaxAngularRate * 0.02) // Add a 2% deadband
      .withSteerRequestType(SteerRequestType.MotionMagic)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop

  private int targetTag;
  private Pose2d targetPose;
  private double shootSpeed;
  private Pose2d currentPose;

  private boolean finished = false;
  private boolean timeStampLock = true;
  private double shootTime = 0;
  /** Creates a new ShootPose. */
  public ShootToZone(CommandSwerveDrivetrain drivetrain, Shooter shooter, Intake intake, double MaxSpeed, double MaxAngularRate, DoubleSupplier TranslationX, DoubleSupplier TranslationY) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    this.MaxSpeed = MaxSpeed;
    this.MaxAngularRate = MaxAngularRate;
    this.TranslationX = TranslationX;
    this.TranslationY = TranslationY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakePosition(IntakeConstants.shootPosition);
    targetTag = DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?7:4;
    yawController.setSetpoint(180.0);
    yawController.setTolerance(2.0);
    yawController.enableContinuousInput(-180, 180);
    targetPose = ShooterConstants.aprilTags.getTagPose(targetTag).get().toPose2d();
    timeStampLock = true;
    finished = false;
    currentPose = drivetrain.getState().Pose;
    distanceToTarget = PhotonUtils.getDistanceToPose(currentPose, targetPose);
    //shootSpeed = 228.98*(Math.pow(10.0, 0.2779*distanceToTarget));
    shootSpeed = (228.98*(Math.exp(0.2779*distanceToTarget)))/60;
    //shootSpeed = 1000;
    SmartDashboard.putNumber("setShootSpeed", shootSpeed);
    shooter.setShooterSpeeds(shootSpeed, 0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPose = drivetrain.getState().Pose;
    rotationToTarget = PhotonUtils.getYawToPose(currentPose, targetPose);
    SmartDashboard.putNumber("Yaw To Target", rotationToTarget.getDegrees());
    SmartDashboard.putNumber("Distance To Target", distanceToTarget);

    double yawSpeed = yawController.calculate(-rotationToTarget.getDegrees());
    yawSpeed = MathUtil.clamp(yawSpeed, -3.8, 3.8);

    drivetrain.setControl(drive.withRotationalRate(yawSpeed).withVelocityX(TranslationX.getAsDouble()).withVelocityY(TranslationY.getAsDouble()));

    shooter.log("isYawAtPosition", yawController.atSetpoint());
    intake.log("isIntakeAtPosition", intake.isIntakeAtPosition(IntakeConstants.shootPosition));
    shooter.log("isShooterAtSpeed", shooter.isShooterAtSpeed(shootSpeed, 0.0));
    shooter.log("LobSpeed", shootSpeed);

    if(yawController.atSetpoint() && intake.isIntakeAtPosition(IntakeConstants.shootPosition) && shooter.isShooterAtSpeed(shootSpeed, 0.0)) {
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
    shooter.stopShooter();
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
