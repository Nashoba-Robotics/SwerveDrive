package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.TestSwerveModule;

public class SwerveTestCommand extends CommandBase{
    double turn;
    double lastTurn = 0;
    @Override
    public void initialize(){
        SmartDashboard.putNumber("Turny", 0);
        SmartDashboard.putNumber("Last Turn", 0);
        //TestSwerveModule.getInstance().set(0);
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("Motor Pos", TestSwerveModule.getInstance().getTurnPos());
        SmartDashboard.putNumber("Abs Pos", TestSwerveModule.getInstance().getAbsPos());
        
        turn = SmartDashboard.getNumber("Turny", 0);
        lastTurn = SmartDashboard.getNumber("Last Turn", 0);
        if(turn != lastTurn){ 
            //TestSwerveModule.getInstance().optimizeSet(0, turn, lastTurn);
            //TestSwerveModule.getInstance().setPos(turn, lastTurn);
            //TestSwerveModule.getInstance().dumbSet(turn);
            //TestSwerveModule.getInstance().testSet(turn, lastTurn);
            TestSwerveModule.getInstance().jank(turn, lastTurn);
            //SmartDashboard.putNumber("Target", TestSwerveModule.getInstance().fixedLowestAngle(turn, lastTurn));
        }
        // lastTurn = TestSwerveModule.getInstance().findLowestAngle(turn, lastTurn);
        // SmartDashboard.putNumber("Lowest Angle", lastTurn);
    }

    @Override
    public void end(boolean interruptible){
        TestSwerveModule.getInstance().stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
