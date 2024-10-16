// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootCommands;

import java.util.function.DoubleSupplier;

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
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootPose extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Shooter shooter;
  private final Intake intake;

  private final PIDController yawController = new PIDController(0.1, 0, 0);
  private final PIDController distanceController = new PIDController(3.5, 0, 0.02);

  private final DoubleSupplier maxSpeed;

  private Rotation2d rotationToTarget;
  private double distanceToTarget;

  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

  private int targetTag;
  private Pose2d targetPose;

  private boolean finished = false;
  private boolean timeStampLock = true;
  private double shootTime = 0;
  /** Creates a new ShootPose. */
  public ShootPose(CommandSwerveDrivetrain drivetrain, Shooter shooter, Intake intake, DoubleSupplier maxSpeed) {
    this.drivetrain = drivetrain;
    this.shooter = shooter;
    this.intake = intake;
    this.maxSpeed = maxSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterSpeeds(ShooterConstants.shootingRPS, 0.05);
    intake.setIntakeRollerCurrentLimit(100);
    intake.setIntakePosition(IntakeConstants.shootPosition);
    targetTag = DriverStation.getAlliance().get()==DriverStation.Alliance.Blue?7:4;
    yawController.setSetpoint(180.0);
    yawController.setTolerance(2.0);
    yawController.enableContinuousInput(-180, 180);
    distanceController.setSetpoint(3.0); //2.69
    distanceController.setTolerance(Units.inchesToMeters(2.0));
    targetPose = ShooterConstants.aprilTags.getTagPose(targetTag).get().toPose2d();
    timeStampLock = true;
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double topSpeed = 4.0;
    if(maxSpeed.getAsDouble() == TunerConstants.kSpeedAt12VoltsMps) {
      topSpeed = 4.0;
    }else{
      topSpeed = maxSpeed.getAsDouble();
    }

    Pose2d currentPose = drivetrain.getState().Pose;
    rotationToTarget = PhotonUtils.getYawToPose(currentPose, targetPose);
    distanceToTarget = PhotonUtils.getDistanceToPose(currentPose, targetPose);

    double yawSpeed = yawController.calculate(-rotationToTarget.getDegrees());
    yawSpeed = MathUtil.clamp(yawSpeed, -3.8, 3.8);
    double distanceSpeed;
    if(Math.abs(yawController.getPositionError()) < 10.0) {
      distanceSpeed = distanceController.calculate(distanceToTarget); 
      distanceSpeed = MathUtil.clamp(distanceSpeed, -topSpeed, topSpeed);
    }else{
      distanceSpeed = 0.0;
    }

    drivetrain.setControl(drive.withRotationalRate(yawSpeed).withVelocityX(distanceSpeed).withVelocityY(0));

    shooter.log("isYawAtPosition", yawController.atSetpoint());
    shooter.log("isDistanceAtPosition", distanceController.atSetpoint());
    intake.log("isIntakeAtPosition", intake.isIntakeAtPosition(IntakeConstants.shootPosition));
    shooter.log("isShooterAtSpeed", shooter.isShooterAtSpeed(ShooterConstants.shootingRPS, 0.05));

    var speeds = drivetrain.getState().speeds;
    double speed = Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
    if(yawController.atSetpoint() && distanceController.atSetpoint() && intake.isIntakeAtPosition(IntakeConstants.shootPosition) && shooter.isShooterAtSpeed(ShooterConstants.shootingRPS, 0.05) && speed < 1.0) {
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
    intake.setIntakeRollerCurrentLimit(IntakeConstants.rollerMotorCurrentLimit);
    shooter.stopShooter();
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
