package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSwerveModule;

public class ToZeroCommand extends CommandBase{
    @Override
    public void initialize(){
        TestSwerveModule.getInstance().toZero();
    }
}
