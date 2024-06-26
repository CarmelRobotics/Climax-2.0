// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import RockinLib.LED.RockinBlinkin;
import RockinLib.LED.RockinLED;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Climber.*;
import frc.robot.Shooter.*;
import frc.robot.Shooter.Shooter.PivotState;
import frc.robot.Intake.Intake;
import frc.robot.Intake.RunIntake;
import frc.robot.Intake.runIntakeforTime;
import frc.robot.Intake.Intake.IntakeState;
import frc.robot.Misc.LED_VIBE;
import frc.robot.generated.TunerConstants;



public class RobotContainer {
  DriverStation drivestation;
  
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  //setup paths 
 // private PathPlannerAuto m_selectedauto = new PathPlannerAuto("2NoteSpeakerMid");
  //private Pose2d startingPose2d = PathPlannerAuto.getStaringPoseFromAutoFile("2noteSpeakerMid");

  




  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandJoystick leftJoystick = new CommandJoystick(0); // Left joystick
    private final CommandJoystick rightJoystick = new CommandJoystick(1); // Right Joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final CommandXboxController controller = new CommandXboxController(4);
  private final CommandJoystick m_guitar =
      new CommandJoystick(OperatorConstants.GUITAR_PORT);
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final DigitalInput beamBreak = new DigitalInput(Constants.BlinkinConstants.NOTE_PORT);
  public  final RockinLED ledManager = new RockinLED(new RockinBlinkin(), beamBreak);
  private final Shooter shooter = new Shooter(drivetrain);
  Command oneNote;
  Command FourNote;
  Command TwoNoteAmp;
  Command TwoNoteSource;
  Command ThreeNoteAmp;

  //setup pose2d presets
  Pose2d blueCenter = new Pose2d(1.35,5.5,Rotation2d.fromDegrees(0));
  Pose2d blueAmp = new Pose2d(0.7,6.7,Rotation2d.fromDegrees(60));
  Pose2d blueSource = new Pose2d(0.75,4.45,Rotation2d.fromDegrees(-60));
  Pose2d redCenter = new Pose2d(15.15,5.5,Rotation2d.fromDegrees(180));
  Pose2d redAmp = new Pose2d(15.85,6.75,Rotation2d.fromDegrees(-60));
  Pose2d redSource = new Pose2d(15.85,4.45,Rotation2d.fromDegrees(60));
  
  
  private final BTS bts = new BTS();
  private final Intake intakemaxxxer = new Intake(beamBreak);
  private final Climber climberLeft = new Climber(true);
  private final Climber climberRight = new Climber(false);
  private String selectedAuto;
  private Command autoCommand;
  private PathPlannerPath twonote1 = PathPlannerPath.fromPathFile("2note1");
  private PathPlannerPath twonote2 = PathPlannerPath.fromPathFile("2note2");
  private PathPlannerPath threenote1 = PathPlannerPath.fromPathFile("3notemid3");
  private PathPlannerPath threenote2 = PathPlannerPath.fromPathFile("3notemid4");
  private PathPlannerPath fournote1 = PathPlannerPath.fromPathFile("4note1");
  private PathPlannerPath fournote2 = PathPlannerPath.fromPathFile("4note2");
  private PathPlannerPath amp1 = PathPlannerPath.fromPathFile("AmpSide1");
  private PathPlannerPath amp2 = PathPlannerPath.fromPathFile("AmpSide2");
  private PathPlannerPath source1 = PathPlannerPath.fromPathFile("sourceside1");
  private PathPlannerPath source2 = PathPlannerPath.fromPathFile("sourceside2");
  private PathPlannerPath ThreeAmp1 = PathPlannerPath.fromPathFile("3NoteAmp1");
  private PathPlannerPath ThreeAmp2 = PathPlannerPath.fromPathFile("3NoteAmp2");
  private PathPlannerPath ThreeAmp3 = PathPlannerPath.fromPathFile("3NoteAmp3");
  private PathPlannerPath ThreeAmp4 = PathPlannerPath.fromPathFile("3NoteAmp4");
  
  private void configureBindings() {
    oneNote = new ParallelCommandGroup(new AutoShoot(shooter, 1,bts));
      // NamedCommands.registerCommand("intake", new RunIntake(intakemaxxxer, -1));
      // NamedCommands.registerCommand("shoot",new AutoShoot(shooter, 1,bts));
      // NamedCommands.registerCommand("SetPose", drivetrain.setPose(startingPose2d));
      // NamedCommands.registerCommand("stop", drivetrain.stop());
      

    //controlling via joystick  
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-leftJoystick.getY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-leftJoystick.getX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-rightJoystick.getX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));


    //controlling via xbox
    //  drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
    //     drivetrain.applyRequest(() -> drive.withVelocityX(-controller.getLeftY() * MaxSpeed) // Drive forward with
    //                                                                                        // negative Y (forward)
    //         .withVelocityY(-controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //         .withRotationalRate(-controller.getRightX() * MaxAngularRate)
    //         .withDeadband(0.1) // Drive counterclockwise with negative X (left)
    //     ));
      


    

    leftJoystick.button(5).whileTrue(drivetrain.applyRequest(() -> brake));
   // leftJoystick.button(6).whileTrue(drivetrain
       // .applyRequest(() -> point.withModuleDirection(new Rotation2d(-leftJoystick.getY(), -leftJoystick.getX()))));

    // reset the field-centric heading on left bumper press
    leftJoystick.button(5).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    // rightJoystick.button(2).whileTrue(new ParallelCommandGroup(new RunIntake(intakemaxxxer, -0.69,bts), new RunBTS(bts, 0.15)));
    leftJoystick.button(2).whileTrue(new ParallelCommandGroup(new RunBTS(bts,-1), new ShootNote(shooter, -0.45)));
    //m_controller2.button(2).toggleOnTrue(new ZeroGyro(drivebase));
    leftJoystick.button(1).whileTrue(new RunIntake(intakemaxxxer, -1,bts));
    leftJoystick.button(3).onTrue(shooter.setPivotMode(PivotState.SPEAKERAIM));
    leftJoystick.button(4).onTrue(shooter.setPivotMode(PivotState.AMPAIM));
    //leftJoystick.button(5).onTrue(shooter.setPivotMode(PivotState.STOW));
    rightJoystick.button(1).whileTrue(new ParallelCommandGroup(new RunBTS(bts,1), new ShootNote(shooter, 1)));
    rightJoystick.button(2).whileTrue(new ParallelCommandGroup(new RunBTS(bts,1), new ShootNote(shooter, 0.15)));
    leftJoystick.button(9).toggleOnTrue(new LED_VIBE(ledManager));
    rightJoystick.button(5).whileTrue(new RunIntake(intakemaxxxer,1,bts));
    rightJoystick.button(6).whileTrue(new PivotManual(shooter, -1));


    controller.leftTrigger().whileTrue(new RunIntake(intakemaxxxer, -0.5,bts));
    controller.rightTrigger().whileTrue(new ParallelCommandGroup(new RunBTS(bts,1), new ShootNote(shooter, 1)));
    controller.rightBumper().whileTrue(new ParallelCommandGroup(new RunBTS(bts,-1), new ShootNote(shooter, -0.45)));
    controller.leftBumper().whileTrue(new RunIntake(intakemaxxxer, 1,bts));
    controller.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    controller.b().whileTrue(drivetrain.applyRequest(()-> drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0)));
    
    // Empty Bowls Guitar Controls
    m_guitar.button(8).whileTrue(new RunIntake(intakemaxxxer, 1,bts));
    m_guitar.button(9).whileTrue(new RunIntake(intakemaxxxer, -0.5,bts));
    m_guitar.button(1).whileTrue(new ParallelCommandGroup(new RunBTS(bts,1), new ShootNote(shooter, 1)));
    m_guitar.button(2).whileTrue(new ParallelCommandGroup(new RunBTS(bts,-1), new ShootNote(shooter, -1)));
    m_guitar.button(3).whileTrue(new PivotManual(shooter, 1));
    controller.x().whileTrue(new ParallelCommandGroup(new MoveClimber(climberLeft,  1),new MoveClimber(climberRight,  1)));
    controller.y().whileTrue(new ParallelCommandGroup(new MoveClimber(climberLeft,  -1),new MoveClimber(climberRight,  -1)));

    // m_guitar.butto\
    
    //n(1).whileTrue(new MoveClimber(climberLeft,  .5));
    // m_guitar.button(2).whileTrue(new MoveClimber(climberRight,  .5));
    // m_guitar.button(3).whileTrue(new MoveClimber(climberLeft, (.5*-1)));
    // m_guitar.button(4).whileTrue(new MoveClimber(climberRight, (.5*-1)));
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }
  public void setUpAuto(){

    //SmartDashboard.putString("Pose", twonote1.getPreviewStartingHolonomicPose().toString());
    //  FourNote = new SequentialCommandGroup(
    //   drivetrain.setPose(redCenter),
    //   new AutoShoot(shooter, 1,bts),
    //   new WaitCommand(1),
    //   new ParallelRaceGroup(new RunIntake(intakemaxxxer, -1,bts), drivetrain.runPathplannerPathFile(twonote1)),
    //   drivetrain.runPathplannerPathFile(twonote2),
    //   drivetrain.stop(),
    //   new AutoShoot(shooter, 1, bts),
    //   new ParallelRaceGroup(drivetrain.runPathplannerPathFile(fournote1), new RunIntake(intakemaxxxer, -1,bts)),
    //   drivetrain.runPathplannerPathFile(fournote2),
    //   drivetrain.stop(),
    //   new AutoShoot(shooter, 1, bts),
    //   new ParallelRaceGroup(drivetrain.runPathplannerPathFile(threenote1), new RunIntake(intakemaxxxer,-1,bts)),
    //   drivetrain.runPathplannerPathFile(threenote2),
    //   drivetrain.stop(),
    //   new AutoShoot(shooter, 1, bts)
    // );

      TwoNoteAmp = new SequentialCommandGroup(
      drivetrain.setPose(blueAmp),
      new AutoShoot(shooter, 1, bts),
      new ParallelRaceGroup(new RunIntake(intakemaxxxer, -1,bts), drivetrain.runPathplannerPathFile(amp1)),
      drivetrain.runPathplannerPathFile(amp2),
      drivetrain.stop(),
      new AutoShoot(shooter, 1, bts)
    );


     TwoNoteSource = new SequentialCommandGroup(
      drivetrain.setPose(redSource),
      new AutoShoot(shooter, 1, bts),
      new ParallelRaceGroup(new RunIntake(intakemaxxxer, -1,bts), drivetrain.runPathplannerPathFile(source1)),
      drivetrain.runPathplannerPathFile(source2),
      drivetrain.stop(),
      new AutoShoot(shooter, 1, bts)
    );


    ThreeNoteAmp = new SequentialCommandGroup(
       drivetrain.setPose(blueAmp),
      new AutoShoot(shooter, 1, bts),
      new ParallelRaceGroup(new RunIntake(intakemaxxxer, -1,bts), drivetrain.runPathplannerPathFile(ThreeAmp1)),
      drivetrain.runPathplannerPathFile(ThreeAmp2),
      drivetrain.stop(),
      new AutoShoot(shooter, 1, bts),
      new ParallelRaceGroup(new RunIntake(intakemaxxxer, -1,bts), drivetrain.runPathplannerPathFile(ThreeAmp3)),
      drivetrain.runPathplannerPathFile(ThreeAmp4),
      drivetrain.stop(),
      new AutoShoot(shooter, 1, bts)
    );

    selectedAuto = SmartDashboard.getEntry("Auto Chooser").getString("null");

    switch(selectedAuto){
        case "4note":
          autoCommand = new SequentialCommandGroup(
               drivetrain.setPose(redCenter),
               new AutoShoot(shooter, 1,bts),
               new WaitCommand(1),
               new ParallelRaceGroup(new RunIntake(intakemaxxxer, -1,bts), drivetrain.runPathplannerPathFile(twonote1)),
              drivetrain.runPathplannerPathFile(twonote2),
              drivetrain.stop(),
              new AutoShoot(shooter, 1, bts),
              new ParallelRaceGroup(drivetrain.runPathplannerPathFile(fournote1), new RunIntake(intakemaxxxer, -1,bts)),
              drivetrain.runPathplannerPathFile(fournote2),
              drivetrain.stop(),
              new AutoShoot(shooter, 1, bts),
              new ParallelRaceGroup(drivetrain.runPathplannerPathFile(threenote1), new RunIntake(intakemaxxxer,-1,bts)),
              drivetrain.runPathplannerPathFile(threenote2),
              drivetrain.stop(),
              new AutoShoot(shooter, 1, bts)
    );
          break;
        case "2notesource":
          autoCommand = new SequentialCommandGroup(
                drivetrain.setPose(redSource),
                new AutoShoot(shooter, 1, bts),
                new ParallelRaceGroup(new RunIntake(intakemaxxxer, -1,bts), drivetrain.runPathplannerPathFile(source1)),
                drivetrain.runPathplannerPathFile(source2),
                drivetrain.stop(),
                new AutoShoot(shooter, 1, bts)
    );
          break;
        case "2noteamp":
          autoCommand = new SequentialCommandGroup(
              drivetrain.setPose(blueAmp),
              new AutoShoot(shooter, 1, bts),
              new ParallelRaceGroup(new RunIntake(intakemaxxxer, -1,bts), drivetrain.runPathplannerPathFile(amp1)),
              drivetrain.runPathplannerPathFile(amp2),
              drivetrain.stop(),
              new AutoShoot(shooter, 1, bts)
    );
          break;
        case "3noteamp":
          autoCommand = new SequentialCommandGroup(
                drivetrain.setPose(blueAmp),
                new AutoShoot(shooter, 1, bts),
                new ParallelRaceGroup(new RunIntake(intakemaxxxer, -1,bts), drivetrain.runPathplannerPathFile(ThreeAmp1)),
                drivetrain.runPathplannerPathFile(ThreeAmp2),
                drivetrain.stop(),
                new AutoShoot(shooter, 1, bts),
                new ParallelRaceGroup(new RunIntake(intakemaxxxer, -1,bts), drivetrain.runPathplannerPathFile(ThreeAmp3)),
                drivetrain.runPathplannerPathFile(ThreeAmp4),
                drivetrain.stop(),
                new AutoShoot(shooter, 1, bts)
    );
          break;
        case "none":
          autoCommand = null;
          break;
        default:
          autoCommand = oneNote;
          break;





    }
    
  }

  public RobotContainer() {
    configureBindings();
    setUpAuto();
    
  }

  public Command getAutonomousCommand() {
    SmartDashboard.putString("selected auto", autoCommand.toString());
    return autoCommand;
  }
}
