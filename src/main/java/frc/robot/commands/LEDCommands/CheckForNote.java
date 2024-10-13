// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class CheckForNote extends Command {
  private final LEDs leds;
  double measurement = 0;
  /** Creates a new LEDCommand. */
  public CheckForNote(LEDs leds) {
    this.leds = leds;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    measurement = leds.getLasercanMeasurement();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(measurement <= 50 && measurement != -1.0) {
      return true;
    }else{
      return false;
    }
  }
}
