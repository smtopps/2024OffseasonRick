// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrapAmpCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trap;
import frc.robot.commands.IntakeCommands.IntakeDownClimb;
import frc.robot.commands.IntakeCommands.IntakeUpClimb;
import frc.robot.constants.ElevatorConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrapSequence extends SequentialCommandGroup {
  /** Creates a new TrapScoring. */
  public TrapSequence(Elevator elevator, Intake intake, Shooter shooter, Trap trap) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ElevatorPosition(elevator, ElevatorConstants.elevatorHandoffPosition),
      new TrapHandoff(intake, shooter, elevator, trap),
      new IntakeDownClimb(intake),
      new ElevatorPosition(elevator, ElevatorConstants.trapPosition),
      new DepositTrapNote(elevator, trap),
      new IntakeUpClimb(intake),
      new ElevatorPosition(elevator, ElevatorConstants.elevatorStowedPosition)
    );
  }
}