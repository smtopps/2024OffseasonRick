// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrapAmpCommands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PIDToPose;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trap;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAmpSequence extends SequentialCommandGroup {
  Pose2d blueAmpPose = new Pose2d(1.83, 7.62, Rotation2d.fromDegrees(-90.0));
  Pose2d redAmpPose = new Pose2d(14.72, 7.62, Rotation2d.fromDegrees(-90.0));
  /** Creates a new AutoAmp. */
  public AlignAmpSequence(Intake intake, CommandSwerveDrivetrain drivetrain, Elevator elevator, Shooter shooter, Trap trap) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(new PIDToPose(drivetrain, blueAmpPose, false), new PIDToPose(drivetrain, redAmpPose, true), ()-> DriverStation.getAlliance().get().equals(Alliance.Blue)),
      new ParallelRaceGroup(
        new InstantCommand(()-> drivetrain.setControl(new SwerveRequest.RobotCentric().withVelocityX(-0.6))),
        new WaitCommand(0.3)
      ),
      new AmpSequence(elevator, intake, shooter, trap)
    );
  }
}
