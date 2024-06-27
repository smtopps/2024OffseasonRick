// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightGamePiece extends SubsystemBase {
  CommandSwerveDrivetrain drivetrain;
  private String ll = "limelight-intake";
  private final Pose3d robotToCamera = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
  /** Creates a new LimelightGamePiece. */
  public LimelightGamePiece() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void updateLimelightResults() {}
}
