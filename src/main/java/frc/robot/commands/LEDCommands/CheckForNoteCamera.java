// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.LEDs;

public class CheckForNoteCamera extends Command {
  private final LEDs leds;
  double measurement = 0;
  /** Creates a new CheckForNoteCamera. */
  public CheckForNoteCamera(LEDs leds) {
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
    if(LimelightHelpers.getTV("limelight-intake")) {
      leds.setLED(0, 0, 255, 8, 64);
    }else{
      leds.setLED(127, 0, 255, 8, 64);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(measurement < 50 && measurement != -1) {
      return true;
    }else{
      return false;
    }
  }
}
