// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShootCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ShootDeflect extends Command {
  private final Shooter shooter;
  private final Intake intake;
  private final Elevator elevator;
  private boolean finished = false;
  private boolean timeStampLock = true;
  private double shootTime = 0;
  /** Creates a new ShootDeflect. */
  public ShootDeflect(Shooter shooter, Intake intake, Elevator elevator) {
    this.shooter = shooter;
    this.intake = intake;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, intake, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterSpeeds(ShooterConstants.deflectRPS, 0.0);
    intake.setIntakePosition(IntakeConstants.shootPosition);
    elevator.setElevatorPosition(ElevatorConstants.ShootDeflect);
    timeStampLock = true;
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.log("isIntakeAtPosition", intake.isIntakeAtPosition(IntakeConstants.shootPosition));
    shooter.log("isShooterAtSpeed", shooter.isShooterAtSpeed(ShooterConstants.deflectRPS, 0.0));
    elevator.log("isElevatorAtPosition", elevator.isElevatorAtPosition(ElevatorConstants.ShootDeflect));
    if(shooter.isShooterAtSpeed(ShooterConstants.deflectRPS, 0.0) && elevator.isElevatorAtPosition(ElevatorConstants.ShootDeflect) && intake.isIntakeAtPosition(IntakeConstants.shootPosition)) {
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
    shooter.stopShooter();
    elevator.setElevatorPosition(ElevatorConstants.elevatorStowedPosition);
    finished = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
