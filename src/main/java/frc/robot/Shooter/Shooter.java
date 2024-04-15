package frc.robot.Shooter;

import java.sql.Driver;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.jni.CANSparkMaxJNI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.RockinTalon;
import frc.robot.Constants.FieldConstants;
import frc.robot.Misc.LED.STATUS;
import frc.robot.Misc.*;

public class Shooter extends SubsystemBase {
    private RockinTalon shootmotorone;
    private RockinTalon shootmotortwo;
    private CANSparkMax pivotmotorone;
    //private AHRS navx = new AHRS(I2C.Port.kOnboard);
    public double currentAngle;
   // private double amountMove;
    private double targetAngle;

    //private LED l;
    private ProfiledPIDController pivotController;
    private ArmFeedforward ffController;
    private CANcoder encoder;
    private Vision limelight;
    private double encoderOffset = 153;
    private CommandSwerveDrivetrain swerve;
    private Rotation2d pivotGoal = Rotation2d.fromDegrees(45);
    private PivotState pivotState = PivotState.STOW;
    private ShooterState shooterState = ShooterState.DEFAULT;
    public double secondsPerDegree;
    private CurrentLimitsConfigs currentLimit = new CurrentLimitsConfigs();
    private TalonFXConfiguration config;
    private double pidOutput;
    private double ffOutput;
    public Shooter(CommandSwerveDrivetrain s){
       currentLimit.withSupplyCurrentLimit(30);
        encoder = new CANcoder(21);
        config.withCurrentLimits(currentLimit);
        swerve = s;
        shootmotorone = new RockinTalon(frc.robot.Constants.Shooter.SHOOTER_MOTORONE_CAN);
        shootmotortwo = new RockinTalon(frc.robot.Constants.Shooter.SHOOTER_MOTORTWO_CAN);
        shootmotorone.setInverted(false);
        shootmotortwo.setInverted(false);
        shootmotorone.getConfigurator().apply(config);
        shootmotortwo.getConfigurator().apply(config);
        limelight = new Vision();
        pivotmotorone = new CANSparkMax(frc.robot.Constants.Shooter.SHOOTER_PIVOTONE_CAN, MotorType.kBrushless);
        pivotmotorone.setSmartCurrentLimit(frc.robot.Constants.Shooter.PIVOT_CURRENT_LIMIT);
        pivotController = Constants.Shooter.SHOOTER_PID_CONTROLLER;
        ffController = Constants.Shooter.SHOOTER_FF_CONTROLLER;

    }
    @Override
    public void periodic(){
       SmartDashboard.putNumber("Pivot angle", getPivotAngle());
       SmartDashboard.putNumber("Goal angle", pivotGoal.getDegrees());
       SmartDashboard.putNumber("PID val", pidOutput);
       SmartDashboard.putNumber("FF output", ffOutput);
       SmartDashboard.putNumber("Manual output", pivotmotorone.get());
      // SmartDashboard.putNumber("Shooter Speed", getAverageRollerSpeed());
       SmartDashboard.putString("Current State",pivotState.toString());
       calcAndApplyControllers();
    //    if(limitswitch.get()){
    //     setMode(ShooterState.ERROR);
    //    }
       if(shootmotorone.get() == 0 || shootmotorone.get() == -0.02){
        shootmotorone.set(-0.02);
        shootmotortwo.set(-0.02);
       }
       switch (pivotState) {
        case SPEAKERSHOOT:
            SpeakerShoot();
            break;
        case TRANSFER:
            transfer();
            break;
        case STOW:
            stow();
            break;
        case ERROR:
            error();
            break;
        case AMPAIM:
            pivotToAngle(Constants.FieldConstants.AMP_ANGLE);
            break;
        case SPEAKERAIM:
            pivotToAngle(getSpeakerAngle(swerve));
            break;
        default:
            break;
       }
       
       
    }
    //state enums
    public static enum PivotState{
        SPEAKERSHOOT,
        TRANSFER,
        STOW,
        ERROR,
        AMPAIM,
        SPEAKERAIM
    }
    public static enum ShooterState{
        SHOOTING,
        AMP,
        DEFAULT
    }
    public void setPivot(PivotState state){
        this.pivotState = state;
    }
    public void setShoot(ShooterState state){
        this.shooterState = state;
    }
    //commands for shooter functoins
    public Command setPivotMode(PivotState state){
        return run(() -> setPivot(state));
    }
    public Command setShooterMode(ShooterState state){
        return run(() -> setShoot(state));
    }
    public Command shootNote(){
        return run(() -> shoot(1));
    }

    //calculates outputs based on desired degree
    public void calcAndApplyControllers(){
        pidOutput = -pivotController.calculate(getPivotAngle(), pivotGoal.getDegrees());
        State profSetpoint = pivotController.getSetpoint();
        ffOutput = -ffController.calculate(profSetpoint.position, profSetpoint.velocity);
        SmartDashboard.putNumber("Output to Pivot", pidOutput + ffOutput);
       // pivotmotorone.set(pidOutput + ffOutput);
       // ^ uncomment above to make it pivot 
    }
    //used for going under stage
    public void stow(){
        pivotToAngle(45);
    }
    //no use this
    public void SpeakerShoot(){
        autoShoot();
    }
    //intake -> shooter
    public void transfer(){
        pivotToAngle(45);
    }
    //if something goofy happens
    public void error(){
        //do the LED error signal when LED code ready
        pivot(0);
        
    }  

    //gets the current pivot angle from the CanCoder
    public double getPivotAngle(){
       return (encoder.getAbsolutePosition().getValue() - encoderOffset);
    }
    //public double getNavxPitch(){
      //  return navx.getPitch();
    //}
    // shoots a note
    public void shoot(double speed){
        // if (speed != 0) {
        //     l.setMode(STATUS.SHOOTING);
        // } else {
        //     l.setMode(STATUS.DEFAULT);
        // }
    
        shootmotorone.set(speed);
        shootmotortwo.set(speed);
    }
    //used for setting pivot 
    public void pivot(double speed){
       pivotmotorone.set(speed);
    }
    // a bunch of unused methods 
    public double getPosition(){
        return pivotmotorone.getEncoder().getPosition();
    }
    public double getDegree(){
        return currentAngle;
    }

    // wonder what this does
    public double getPivotVelocity(){
        return (encoder.getVelocity().getValue()) * (2*Math.PI);
    }
    // public void setAngle(double degree){
    //     currentAngle = degree;`
        
    // }

    //sets the goal angle
    public void pivotToAngle(double angle){
        pivotGoal = Rotation2d.fromDegrees(angle);
        pivotController.reset(Rotation2d.fromDegrees(getPivotAngle()).getRadians(),getPivotVelocity());
    }
    public boolean isFalling(){
        return ((pivotmotorone.getOutputCurrent() == 0));
    }
    //some more unused methods
    public void splinePivot(){

    }
    
    public void autoShoot(){
        pivotToAngle(getSpeakerAngle(swerve));
        shoot(1);
    }
    public void AutoShootAmp(){
        pivotToAngle(FieldConstants.AMP_ANGLE);
        shoot(1);
    }
    public double getSpeakerAngle(CommandSwerveDrivetrain drive){
        double x;
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
             x = drive.getPose().getX() - Constants.FieldConstants.SPEAKER_X_RED;
        } else{
             x = drive.getPose().getX();
        }
        
        double y = 5.5 - drive.getPose().getY();
        double distFromSpeaker  = Math.sqrt((Math.pow(x,2)) + (Math.pow(y, 2)));
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){
        return Math.atan(Constants.FieldConstants.SPEAKER_HEIGHT / limelight.getPoseX() - Constants.FieldConstants.SPEAKER_X_BLUE);
        } else {
            return Math.atan(Constants.FieldConstants.SPEAKER_HEIGHT / limelight.getPoseX() - Constants.FieldConstants.SPEAKER_X_RED);
        }
    }
}
