// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDCommands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDs;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LEDCommand extends SequentialCommandGroup {
  /** Creates a new LEDCommand. */
  public LEDCommand(LEDs leds, GenericHID controller) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()-> leds.setLED(255, 0, 0, 8, 64), leds),
      new CheckForNote(leds),
      new InstantCommand(()-> leds.setLED(255, 255, 255, 8, 64), leds),
      new WaitCommand(0.15),
      new InstantCommand(()-> leds.setLED(0, 0, 0, 0, 64), leds),
      new WaitCommand(0.15),
      new InstantCommand(()-> leds.setLED(255, 255, 255, 8, 64), leds),
      new WaitCommand(0.15),
      new InstantCommand(()-> leds.setLED(0, 0, 0, 0, 64), leds),
      new WaitCommand(0.15),
      new InstantCommand(()-> leds.setLED(255, 255, 255, 8, 64), leds),
      new WaitCommand(0.15),
      new InstantCommand(()-> leds.setLED(0, 0, 0, 0, 64), leds),
      new WaitCommand(0.15),
      new InstantCommand(()-> leds.setLED(255, 255, 255, 8, 64), leds),
      new WaitCommand(0.15),
      new InstantCommand(()-> leds.setLED(0, 0, 0, 0, 64), leds),
      new WaitCommand(0.15),
      new InstantCommand(()-> leds.setLED(0, 255, 0, 8, 64), leds),
      new CheckForNoteGone(leds),
      new InstantCommand(()-> leds.setLED(255, 0, 0, 8, 64), leds)
    );
  }
}
