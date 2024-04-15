package RockinLib.Control;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RockinJoystick extends CommandJoystick{
    public RockinJoystick(int port){
        super(port);
    }
    public Trigger Trigger(){
        return this.button(1);
    }
    public Trigger ThumbButton(){
        return this.button(2);
    }
    public Trigger DpadUp(){
        return this.pov(0);
    }
    public Trigger DpadDown(){
        return this.pov(180);
    }
    public Trigger DpadRight(){
        return this.pov(90);
    }
    public Trigger DpadLeft(){
        return this.pov(270);
    }

    //I forgot the numbers of these buttons so these are incomplete
    public Trigger UpperLeft(){
        return null;
    }
    public Trigger UpperRight(){
        return null;
    }
    public Trigger LowerLeft(){
        return null;
    }
    public Trigger LowerRight(){
        return null;
    }



}
