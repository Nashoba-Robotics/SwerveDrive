package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.util.Units;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.TestSwerveModule;

public class SwerveTestCommand extends CommandBase{
    SwerveModule mod = new SwerveModule(Constants.SwerveBase.frontRightMovePort, 
                                        Constants.SwerveBase.frontRightTurnPort, 
                                        Constants.SwerveBase.frontRightSensorPort, 
                                        0);
    
    double turn;
    double lastTurn = 0;

    @Override
    public void initialize(){
        SmartDashboard.putNumber("Turny", 0);
        SmartDashboard.putNumber("Last Turn", 0);

        TestSwerveModule.getInstance().set(0);

        SmartDashboard.putNumber("Motor Pos", TestSwerveModule.getInstance().getTurnPos());
        SmartDashboard.putNumber("Abs Pos", TestSwerveModule.getInstance().getAbsPos());
        // SmartDashboard.putNumber("Motor Pos", mod.getMotorPos());
        // SmartDashboard.putNumber("Abs Pos", mod.getSensorPos());
    }

    @Override
    public void execute(){
        // SmartDashboard.putNumber("Motor Pos", TestSwerveModule.getInstance().getTurnPos());
        // SmartDashboard.putNumber("Abs Pos", TestSwerveModule.getInstance().getAbsPos());
        //double motorPos = mod.getMotorPos();
        // SmartDashboard.putNumber("Motor Pos", motorPos);
        // SmartDashboard.putNumber("Motor Angle", Units.NUToDeg(motorPos));
        // SmartDashboard.putNumber("Abs Pos", mod.getSensorPos());
        
        turn = SmartDashboard.getNumber("Turny", 0);
        lastTurn = SmartDashboard.getNumber("Last Turn", 0);

        double currentPos = TestSwerveModule.getInstance().fixedLowestAngle(turn, Units.NUToDeg(TestSwerveModule.getInstance().getMotorPos()));
        SmartDashboard.putNumber("Desired Angle", currentPos);
        SmartDashboard.putNumber("Current Pos", currentPos);

        //mod.set(0, turn);
        TestSwerveModule.getInstance().testSet(turn);

        if(turn != lastTurn){ 
            //TestSwerveModule.getInstance().optimizeSet(0, turn, lastTurn);
            //TestSwerveModule.getInstance().setPos(turn, lastTurn);
            //TestSwerveModule.getInstance().dumbSet(turn);
            //TestSwerveModule.getInstance().jank(turn, lastTurn);
            //SmartDashboard.putNumber("Target", TestSwerveModule.getInstance().fixedLowestAngle(turn, lastTurn));
        }
        //lastTurn = TestSwerveModule.getInstance().findLowestAngle(turn, lastTurn);
        //SmartDashboard.putNumber("Lowest Angle", lastTurn);
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
