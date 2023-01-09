package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveModule;
import frc.robot.Constants;

public class OffsetTestCommand extends CommandBase{
    SwerveModule mod = new SwerveModule(Constants.SwerveBase.frontRightMovePort, 
                                        Constants.SwerveBase.frontRightTurnPort,
                                        Constants.SwerveBase.frontRightSensorPort,
                                         77);

    @Override
    public void initialize() {
        mod.zero();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Motor Pos", mod.getMotorPos());
        SmartDashboard.putNumber("CanCoder Pos", mod.getSensorPos());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
