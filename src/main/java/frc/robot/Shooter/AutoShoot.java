package frc.robot.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.Shooter.ShooterState;

public class AutoShoot extends Command {
    // Called once the command ends or is interrupted.
    Shooter shooter;
    BTS bts;
    double speed;
    Timer timey;
    public AutoShoot(Shooter s, double x, BTS b){
        shooter = s;
        bts = b;
        timey = new Timer();
        speed = x;
    }
    @Override
    public void initialize(){
        timey.start();
        //shooter.setShoot(ShooterState.SHOOTING);
    }
    @Override
    public void execute(){
        shooter.shoot(-speed);
        bts.set(1);
    }
    @Override
    public void end(boolean interrupted)
    {
        shooter.shoot(0);
        bts.set(0);
        //shooter.setShoot(ShooterState.DEFAULT);
        timey.stop();
        timey.reset();
    }

    // Returns true when the command should end.
  @Override
    public boolean isFinished()
    {
     return timey.get() > 1;
     } 
}   
