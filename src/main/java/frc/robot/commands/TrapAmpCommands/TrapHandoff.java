// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrapAmpCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trap;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TrapConstants;

public class TrapHandoff extends Command {
  private final Intake intake;
  private final Shooter shooter;
  private final Elevator elevator;
  private final Trap trap;
  private double handoffTime;

  /** Creates a new TrapHandoff. */
  public TrapHandoff(Intake intake, Shooter shooter, Elevator elevator, Trap trap) {
    this.intake = intake;
    this.shooter = shooter;
    this.elevator = elevator;
    this.trap = trap;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter, elevator, trap);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterSpeeds(ShooterConstants.trapHandoffRPS, 0.0);
    intake.setIntakePosition(IntakeConstants.shootPosition);
    intake.setRollerSpeed(IntakeConstants.trapSpeed);
    elevator.setElevatorPosition(ElevatorConstants.elevatorHandoffPosition);
    trap.setTrapSpeed(TrapConstants.trapHandoffSpeed);
    handoffTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(IntakeConstants.stowedPosition);
    intake.setRollerSpeed(IntakeConstants.stallRPM);
    trap.stopTrapMotor();
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() - handoffTime > TrapConstants.trapHandoffTime) {
      return true;
    }else{
      return false;
    }
  }
}
