package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.lib.util.SwerveState;
import frc.robot.subsystems.SwerveModule1;

public class SingleSwerveDriveCommand extends CommandBase{
    SwerveModule1 module;
    double lastTurn = 0;
    double initPos = 0;
    @Override
    public void initialize(){
        module = new SwerveModule1(Constants.SwerveBase.frontRightMovePort, Constants.SwerveBase.frontRightTurnPort, 1);
        //module.reset();

        initPos = module.getTrueTurnPosition();

        SmartDashboard.putNumber("Turn", 0);
        SmartDashboard.putNumber("Move", 0);
        SmartDashboard.putBoolean("maths", false);

        module.dumbSet(new SwerveState(0, 0, lastTurn));
    }

    @Override
    public void execute(){
        double pos = module.getTurnPosition();
        double absPos = module.getTrueTurnPosition();
        // absPos -= initPos;

        SmartDashboard.putNumber("Sensor Position", pos);
        SmartDashboard.putNumber("CANCoder Pos", absPos);
        SmartDashboard.putNumber("Angle", module.NU2Radians(pos));
        double turn = SmartDashboard.getNumber("Turn", 0);
        double move = SmartDashboard.getNumber("Move", 0);

        SwerveState state = new SwerveState(move, turn, lastTurn);
        if(turn != lastTurn){
             module.dumbSet(state);
             lastTurn = turn;
        }

        //lastTurn = turn;
    }

    @Override
    public void end(boolean interruptible){
        //module.reset();
        module.dumbSet(new SwerveState(0, 0, lastTurn));
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
