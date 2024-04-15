package frc.robot.Misc;

import RockinLib.LED.RockinLED;
import edu.wpi.first.wpilibj2.command.Command;

public class LED_VIBE extends Command {
    // Varyables 
    private RockinLED lights;

    public LED_VIBE(RockinLED ligyboi) {
        // Shooter Object
        lights = ligyboi;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        if (lights.getState() != RockinLED.STATUS.VIBE) {
            lights.setMode(RockinLED.STATUS.VIBE); 
        }
    }
    @Override
    public void end(boolean interrupted){
        lights.defaultState();
    }
    @Override
    public boolean isFinished(){return false;}
}
