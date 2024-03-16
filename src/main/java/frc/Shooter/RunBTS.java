package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BTS;
import frc.robot.subsystems.Shooter;

public class RunBTS extends Command {
    // Called once the command ends or is interrupted.
    BTS bts;
    double speed;
    public RunBTS(BTS b, double s){
        bts = b;
        speed = s;
    }
    @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){
        bts.set(speed);
        System.out.println("running bts");
    }
    @Override
    public void end(boolean interrupted)
    {
        bts.set(0);
    }

    // Returns true when the command should end.
  @Override
    public boolean isFinished()
    {
     return false;
     } 
}   
