// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ChangeSpeed;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.AutoCommands.AutoAlignNotes;
import frc.robot.commands.AutoCommands.AutoManualShoot;
import frc.robot.commands.AutoCommands.AutoShootDeflect;
import frc.robot.commands.AutoCommands.AutoShootOnTheFly;
import frc.robot.commands.AutoCommands.AutoIntakeStart;
import frc.robot.commands.AutoCommands.AutoIntakeStop;
import frc.robot.commands.AutoCommands.AutoShootPose;
import frc.robot.commands.IntakeCommands.IntakeAmp;
import frc.robot.commands.IntakeCommands.IntakeGround;
import frc.robot.commands.IntakeCommands.IntakeGroundAlignDrive;
import frc.robot.commands.IntakeCommands.IntakeTryHarder;
import frc.robot.commands.LEDCommands.CheckForNoteCamera;
import frc.robot.commands.LEDCommands.LEDCommand;
import frc.robot.commands.ShootCommands.ManualShoot;
import frc.robot.commands.ShootCommands.RevShooter;
import frc.robot.commands.ShootCommands.ShootDeflect;
import frc.robot.commands.ShootCommands.ShootPose;
import frc.robot.commands.ShootCommands.ShootToZone;
import frc.robot.commands.TrapAmpCommands.AlignAmpSequence;
import frc.robot.commands.TrapAmpCommands.ClimberManual;
import frc.robot.commands.TrapAmpCommands.ElevatorManual;
import frc.robot.commands.TrapAmpCommands.ClimberHold;
import frc.robot.commands.TrapAmpCommands.ElevatorHold;
import frc.robot.commands.TrapAmpCommands.TrapManual;
import frc.robot.commands.TrapAmpCommands.AmpSequence;
import frc.robot.commands.TrapAmpCommands.TrapSequence;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LimelightAprilTag;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Trap;
import monologue.Logged;

public class RobotContainer implements Logged{

  private final SendableChooser<Command> autoChooser;

  public double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0); // My joystick
  private final CommandXboxController operatorController = new CommandXboxController(1);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  private final Elevator elevator = new Elevator();
  private final Trap trap = new Trap();
  private final LimelightAprilTag limelight = new LimelightAprilTag(drivetrain, "limelight-shooter");
  private final LEDs leds = new LEDs();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withSteerRequestType(SteerRequestType.MotionMagic) // I want Motion Magic module steering
      .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric
                                                               // driving in open loop

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    /*drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));*/
    drivetrain.setDefaultCommand(
      new DriveCommand(drivetrain, ()-> driverController.getLeftX(), ()-> driverController.getLeftY(), ()-> driverController.getRightX(), ()-> MaxSpeed, ()-> MaxAngularRate).ignoringDisable(true)
    );

    leds.setDefaultCommand(new LEDCommand(leds, driverController.getHID()).ignoringDisable(true).repeatedly());

    // reset the field-centric heading on left bumper press
    driverController.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative(new Pose2d())).ignoringDisable(true));
    driverController.back().onTrue(new ChangeSpeed(this).ignoringDisable(true));

    drivetrain.registerTelemetry(logger::telemeterize);


    /* Bindings for drivetrain characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    //driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    //driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    //driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    //driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    shooter.setDefaultCommand(new RevShooter(drivetrain, shooter, ()-> MaxSpeed).onlyWhile(()-> DriverStation.isTeleop()));

    driverController.rightBumper().whileTrue(new IntakeGround(intake));
    driverController.rightTrigger(0.05).whileTrue(new IntakeGroundAlignDrive(intake,drivetrain, ()-> driverController.getRightTriggerAxis(),()-> -driverController.getLeftY() * MaxSpeed, ()-> -driverController.getLeftX() * MaxSpeed, ()-> -driverController.getRightX() * MaxAngularRate, drive, ()-> MaxSpeed).deadlineWith(new CheckForNoteCamera(leds)).ignoringDisable(true));
    driverController.leftBumper().whileTrue(new ManualShoot(shooter, intake, ShooterConstants.shootingRPS));
    driverController.leftTrigger().whileTrue(new ShootPose(drivetrain, shooter, intake, ()-> MaxSpeed).deadlineWith(new InstantCommand(()->leds.setLED(0, 0, 255, 8, 64), leds)).ignoringDisable(true));
    driverController.y().whileTrue(new IntakeAmp(intake));
    driverController.b().onTrue(new AmpSequence(elevator, intake, shooter, trap));
    driverController.rightStick().whileTrue(new AlignAmpSequence(intake, drivetrain, elevator, shooter, trap).deadlineWith(new InstantCommand(()->leds.setLED(0, 0, 255, 8, 64), leds)).ignoringDisable(true));
    driverController.a().whileTrue(new ShootDeflect(shooter, intake, elevator));
    driverController.leftStick().whileTrue(new ShootToZone(drivetrain, shooter, intake, MaxSpeed, MaxAngularRate, ()-> -driverController.getLeftY() * MaxSpeed, ()-> -driverController.getLeftX() * MaxSpeed).deadlineWith(new InstantCommand(()->leds.setLED(0, 0, 255, 8, 64), leds)).ignoringDisable(true));

    operatorController.a().whileTrue(new ClimberManual(climber, ()-> -operatorController.getLeftY())).onFalse(new ClimberHold(climber));
    operatorController.y().whileTrue(new ElevatorManual(elevator, ()-> operatorController.getLeftY())).onFalse(new ElevatorHold(elevator));
    operatorController.x().whileTrue(new TrapManual(trap, ()-> operatorController.getLeftY()));
    operatorController.back().onTrue(new InstantCommand(()-> climber.resetEncoder(true), climber));
    operatorController.rightBumper().onTrue(new TrapSequence(elevator, intake, shooter, trap));
    operatorController.rightTrigger().whileTrue(new IntakeTryHarder(intake));
    operatorController.povCenter().whileFalse(new IntakeAmp(intake));
  }

  public RobotContainer() {
    NamedCommands.registerCommand("startIntake", new AutoIntakeStart(intake));
    NamedCommands.registerCommand("stopIntake", new AutoIntakeStop(intake));
    NamedCommands.registerCommand("autoShoot", new AutoShootPose(drivetrain, shooter, intake));
    NamedCommands.registerCommand("autoIntake", new PrintCommand("autoIntake"));
    NamedCommands.registerCommand("autoShootAlign", new PrintCommand("autoShootAlign"));
    NamedCommands.registerCommand("autoAlign", new AutoAlignNotes(drivetrain, intake));
    NamedCommands.registerCommand("deflect", new AutoShootDeflect(shooter, intake, elevator));
    NamedCommands.registerCommand("manuelShoot", new AutoManualShoot(shooter, intake));
    //NamedCommands.registerCommand("manuelShoot", new AutoShootOnTheFly(drivetrain, shooter, intake));
    NamedCommands.registerCommand("moveShoot", new AutoShootOnTheFly(drivetrain, shooter, intake));
    NamedCommands.registerCommand("shootMove", new AutoShootOnTheFly(drivetrain, shooter, intake));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
  }

  public void configureAutoSettings() {
    shooter.setShooterSpeeds(ShooterConstants.shootingRPS, 0.05);
    limelight.useLimelight(true);
  }

  public void configureTeleopSettings() {
    shooter.stopShooter();
    limelight.useLimelight(true);
  }
}
