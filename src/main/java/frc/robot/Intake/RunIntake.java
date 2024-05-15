package frc.robot.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Shooter.BTS;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;

public class RunIntake extends Command {
    // Called once the command ends or is interrupted.
    Intake intake;
    double speed;
    BTS bts;
    public RunIntake(Intake intake, double speed, BTS bts){
        this.intake = intake;
        this.speed = speed;
        this.bts = bts;
    }
    @Override
    public void initialize(){
        Commands.print("Running intake");
    }
    @Override
    public void execute(){
        intake.runIntake(speed);
        bts.set(Math.signum(speed)*0.15);
        //0.4 optimal speed
    }
    @Override
    public void end(boolean interrupted)
    {
        intake.runIntake(0);
        bts.set(0);
    }

    // Returns true when the command should end.
  @Override
    public boolean isFinished()
    {
     return !intake.beamSensor.get();
     } 
}   
