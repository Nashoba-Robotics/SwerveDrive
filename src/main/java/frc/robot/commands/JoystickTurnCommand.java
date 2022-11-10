package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSwerveModule;

//Temporary, I'm just going to make a Joystick object in here
public class JoystickTurnCommand extends CommandBase{
    Joystick joystick = new Joystick(0);
    double lastTurn = 0;
    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double x = joystick.getX();
        double y = joystick.getY();

        //In radians
        double turn =  Math.asin(Math.sqrt(x*x + y*y));

        if(y < 0){
            turn = Math.signum(turn) * (Math.PI - Math.abs(turn));
        }

        //Convert from radians to degrees
        turn *= 360/(2*Math.PI);

        //if(turn != lastTurn) TestSwerveModule.getInstance().optimizeSet(turn, lastTurn);
        lastTurn = TestSwerveModule.getInstance().findLowestAngle(turn, lastTurn);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
