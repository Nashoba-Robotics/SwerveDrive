package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSwerveModule;

//Make a command where the module will spin 360 degrees.
public class CircleSpinCommand extends CommandBase{
    int turn;
    Timer timer;
    int lastTurn;
    int rps = 5;    //Rotations per loop
    double loopTime = 0.5;  //How long is it going to take to increase the rotation by rps

    @Override
    public void initialize() {
        turn = 0;
        lastTurn = 0;
        timer = new Timer();
        timer.start();
    }

    @Override
    public void execute() {
       TestSwerveModule.getInstance().jank(turn-180, lastTurn);
       if(timer.get() > 0.5){
            lastTurn = turn-180;
            turn += rps;
            turn %= 360;
            timer.reset();
       }
    }

    @Override
    public void end(boolean interrupted) {
        TestSwerveModule.getInstance().stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
