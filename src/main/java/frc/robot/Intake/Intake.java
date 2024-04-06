package frc.robot.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
//
public class Intake extends SubsystemBase{
    private CANSparkMax intakemotorOne;
    private IntakeState state = IntakeState.DEFAULT;
    private CANSparkMax intakemotorTwo;
    private AnalogInput distanceSensor;
    public DigitalInput beamSensor;
    
    public Intake(DigitalInput bSensor){
        distanceSensor = new AnalogInput(4);
        intakemotorOne = new CANSparkMax(Constants.Intake.INTAKE_CAN_ONE,MotorType.kBrushless );
        intakemotorTwo = new CANSparkMax(Constants.Intake.INTAKE_CAN_TWO, MotorType.kBrushless );
        beamSensor = bSensor;
        // intakemotorOne.setClosedLoopRampRate(1.0);
        // intakemotorTwo.setClosedLoopRampRate(1.0);
        // intakemotorOne.setIdleMode(IdleMode.kCoast);
        // intakemotorTwo.setIdleMode(IdleMode.kCoast);
        
    }
   @Override
   public void periodic(){
    
   }
    // public void periodic(){
    //     SmartDashboard.putString("Current Intake State", state.toString());
    //     switch(state){
    //         case INTAKING:
    //             runIntake(-0.75);
    //             break;
    //         case OUTTAKING:
    //             runIntake(1);
    //             break;
    //         case TRANSFERING:
    //             runIntake(-1);
    //         default:
    //             runIntake(-0);
    //             break;
            
      //  }
    //}
    public void runIntake(double speed){
        intakemotorOne.set(-speed);
        intakemotorTwo.set(-speed);
        System.out.println("Running Intake");
    }
    public double getDist(){
        return AnalogInput.getGlobalSampleRate();
    }
    public boolean hasNote(){
        return (getDist() < 1.5);
        
    }
    public Command setIntakeState(IntakeState state){
        return run(() -> setState(state));
    }
    public IntakeState setState(IntakeState state){
        if(this.state != IntakeState.DEFAULT){
            this.state = IntakeState.DEFAULT;
            return this.state;
        }
        this.state = state;
        return this.state;
    }
    public static enum IntakeState
    {
        INTAKING,
        OUTTAKING,
        TRANSFERING,
        DEFAULT
    }

}
