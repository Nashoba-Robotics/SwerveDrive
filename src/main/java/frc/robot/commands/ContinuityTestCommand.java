package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSwerveModule;

public class ContinuityTestCommand extends CommandBase{
    double lt = 0;
    @Override
    public void initialize() {
        //TestSwerveModule.getInstance().jank(30, 0);
        SmartDashboard.putNumber("t", 0);
        SmartDashboard.putNumber("lt", 0);
    }

    @Override
    public void execute() {
        double turn = SmartDashboard.getNumber("t", 0);
        double lastTurn = SmartDashboard.getNumber("lt", 0);

        if(lt != turn){
            TestSwerveModule.getInstance().jank(turn, lastTurn);
        }
        lt = turn;
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
