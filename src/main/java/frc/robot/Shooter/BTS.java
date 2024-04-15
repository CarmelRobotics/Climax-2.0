package frc.robot.Shooter;

//import com.ctre.phoenix.platform.DeviceType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import RockinLib.MotorControllers.RockinTalon;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//its BTS but its actually a NTS
//hasty response reference lmao
public class BTS extends SubsystemBase{
    TalonFX BTSMotor;
    TalonFX BTSMotor2 = new TalonFX(61);
    public BTS(){
        BTSMotor = new RockinTalon(60,30);
        BTSMotor2 = new RockinTalon(61,30);

    }
    public void set(double speed){
        BTSMotor.set(-speed);
        BTSMotor2.set(-speed);
    }
    @Override
    public void periodic(){
      
    }
    
    
}
