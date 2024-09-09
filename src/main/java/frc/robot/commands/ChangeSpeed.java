// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.TunerConstants;

public class ChangeSpeed extends Command {
  private final RobotContainer robotContainer;
  /** Creates a new ChangeSpeed. */
  public ChangeSpeed(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(robotContainer.MaxSpeed == TunerConstants.kSpeedAt12VoltsMps) {
      robotContainer.MaxSpeed = 1.2;
    }else{
      robotContainer.MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
