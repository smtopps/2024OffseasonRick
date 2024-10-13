// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Utils.JoystickFilterConfig;
import frc.robot.Utils.PolarJoystickFilter;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveCommand extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier JoystickX, JoystickY, JoystickR, MaxSpeed, MaxAngularRate;
  private final PolarJoystickFilter translationFilter, rotationFilter;

  private final SwerveRequest.FieldCentric drive;
  /** Creates a new DriveCommand. */
  public DriveCommand(CommandSwerveDrivetrain drivetrain, DoubleSupplier JoystickX, DoubleSupplier JoystickY, DoubleSupplier JoystickR, DoubleSupplier MaxSpeed, DoubleSupplier MaxAngularRate) {
    this.drivetrain = drivetrain;
    this.JoystickX = JoystickX;
    this.JoystickY = JoystickY;
    this.JoystickR = JoystickR;
    this.MaxSpeed = MaxSpeed;
    this.MaxAngularRate = MaxAngularRate;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    translationFilter = new PolarJoystickFilter(new JoystickFilterConfig(0.0, 1000.0, 3.0, 1.0));
    rotationFilter = new PolarJoystickFilter(new JoystickFilterConfig(0.0, 1000.0, 3.0, 1.0));
    drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed.getAsDouble() * 0.1).withRotationalDeadband(MaxAngularRate.getAsDouble() * 0.1) // Add a 10% deadband
      .withSteerRequestType(SteerRequestType.MotionMagic) // I want Motion Magic module steering
      .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric
                                                               // driving in open loop
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] translation = translationFilter.filter(JoystickY.getAsDouble(), JoystickX.getAsDouble());

    double translationX = -translation[0] * MaxSpeed.getAsDouble();
    double translationY = -translation[1] * MaxSpeed.getAsDouble();
    double rotation = rotationFilter.filter(-JoystickR.getAsDouble(), 0)[0] * MaxAngularRate.getAsDouble();
    drivetrain.setControl(drive.withVelocityX(translationX).withVelocityY(translationY).withRotationalRate(rotation));

    /*drivetrain.setControl(drive
            .withVelocityX(JoystickY.getAsDouble() * MaxSpeed.getAsDouble()) // Drive forward with negative Y (forward)
            .withVelocityY(-JoystickX.getAsDouble() * MaxSpeed.getAsDouble()) // Drive left with negative X (left)
            .withRotationalRate(-JoystickR.getAsDouble() * MaxAngularRate.getAsDouble()) // Drive counterclockwise with negative X (left)
        );*/
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
