// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Climber.*;
import frc.robot.Shooter.*;
import frc.robot.Shooter.Shooter.PivotState;
import frc.robot.Intake.*;
import frc.robot.Intake.Intake.IntakeState;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandJoystick leftJoystick = new CommandJoystick(0); // Left joystick
    private final CommandJoystick rightJoystick = new CommandJoystick(1); // Right Joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final CommandJoystick m_guitar =
      new CommandJoystick(OperatorConstants.GUITAR_PORT);
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  //public  final LED ledManager = new LED();
  private final Shooter shooter = new Shooter(drivetrain);
  Command oneNote;
  
  private final BTS bts = new BTS();
  private final Intake intakemaxxxer = new Intake();
  private final Climber climberLeft = new Climber(true);
  private final Climber climberRight = new Climber(false);
  

  private void configureBindings() {
    oneNote = new ParallelCommandGroup(new AutoShoot(shooter, 1), new runBTSfortime(bts,1,1));
      NamedCommands.registerCommand("intake", intakemaxxxer.setIntakeState(IntakeState.INTAKING));
      Command intake = NamedCommands.getCommand("intake");
      Command resetGyro = NamedCommands.getCommand("ZeroGyro");
      Command shoot = oneNote;
      Command headingTrue = NamedCommands.getCommand("SetHeadingCorrectionTrue");
      Command headingFalse = NamedCommands.getCommand("SetHeadingCorrectionFalse");
      Command stop = NamedCommands.getCommand("stop");
      
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-leftJoystick.getY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-leftJoystick.getX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-rightJoystick.getX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    leftJoystick.button(5).whileTrue(drivetrain.applyRequest(() -> brake));
   // leftJoystick.button(6).whileTrue(drivetrain
       // .applyRequest(() -> point.withModuleDirection(new Rotation2d(-leftJoystick.getY(), -leftJoystick.getX()))));

    // reset the field-centric heading on left bumper press
    m_guitar.button(5).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    rightJoystick.button(2).whileTrue(new ParallelCommandGroup(new RunIntake(intakemaxxxer, -0.69), new RunBTS(bts, 0.15)));
    leftJoystick.button(2).whileTrue(new RunIntake(intakemaxxxer,1));
    //m_controller2.button(2).toggleOnTrue(new ZeroGyro(drivebase));
    leftJoystick.button(1).whileTrue(new RunIntake(intakemaxxxer, -0.69));
    leftJoystick.button(3).onTrue(shooter.setPivotMode(PivotState.SPEAKERAIM));
    leftJoystick.button(4).onTrue(shooter.setPivotMode(PivotState.AMPAIM));
    leftJoystick.button(5).onTrue(shooter.setPivotMode(PivotState.STOW));
    rightJoystick.button(1).whileTrue(new ParallelCommandGroup(new RunBTS(bts,1), new ShootNote(shooter, 1)));
    rightJoystick.button(3).whileTrue(new ShootNote(shooter, -0.5));
   // m_controller1.button(9).toggleOnTrue(new LED_VIBE(ledManager));
    rightJoystick.button(5).whileTrue(new PivotManual(shooter, 0.75));
    rightJoystick.button(6).whileTrue(new PivotManual(shooter, -0.75));
    m_guitar.button(1).whileTrue(new MoveClimber(climberLeft,  .5));
    m_guitar.button(2).whileTrue(new MoveClimber(climberRight,  .5));
    m_guitar.button(3).whileTrue(new MoveClimber(climberLeft, (.5*-1)));
    m_guitar.button(4).whileTrue(new MoveClimber(climberRight, (.5*-1)));
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
