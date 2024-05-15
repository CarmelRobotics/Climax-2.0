package RockinLib.PID;
import java.time.chrono.HijrahChronology;

import com.ctre.phoenix6.hardware.CANcoder;

import RockinLib.MotorControllers.RockinTalon;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class RockinTalonPIDSubsystem extends SubsystemBase{
    RockinTalon[] talons;
    PIDController pidController;
    CANcoder[] encoder;
    double pidOutput = 0;
    /**
     * subsystembase but if it was good
     * Todo: "finish" and create custom cancoder wrapper
     * @param pidController
     * @param encoder
     * @param talons
     * Have your talons pre-configed
     * @param name
     
    **/
    public RockinTalonPIDSubsystem(String name, PIDController pidController, CANcoder[] encoders, RockinTalon... talons){
        this.talons = talons;
        this.pidController = pidController;
        this.encoder = encoder;
        this.setName(name);
    }

    public void highLevelPeriodic(){

    }
    public void calcAndApply(double setpoint){
        for(int x = 0; x <= talons.length; x++){
         talons[x].set(pidController.calculate(encoder[x].getAbsolutePosition().getValueAsDouble(), setpoint));
        }
    }
    public double getSetpoint(){
        return 0;
    }
    @Override 
    /** 
     * Do not override this method. Instead use {@link RockinTalonPIDSubsystem#highLevelPeriodic()}
     *
    */
    public void periodic(){
        calcAndApply(getSetpoint());
        highLevelPeriodic();
    }

}
