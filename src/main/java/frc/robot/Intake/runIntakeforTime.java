package frc.robot.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Shooter.BTS;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;

public class runIntakeforTime extends Command {
    // Called once the command ends or is interrupted.
    Intake intake;
    double speed;
    Timer timey;
    double time;
    public runIntakeforTime(Intake intake, double s,double time){
        this.intake = intake;
        speed = s;
        timey = new Timer();
        this.time = time;
    }
    @Override
    public void initialize(){
        timey.reset();
        timey.start();
    }
    @Override
    public void execute(){
        intake.runIntake(speed);
        //0.4 optimal speed
    }
    @Override
    public void end(boolean interrupted)
    {
        timey.stop();
        intake.runIntake(0);
    }

    // Returns true when the command should end.
  @Override
    public boolean isFinished()
    {
        return timey.get() > time;
     } 
}   
