package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSwerveModule;

public class ConfigOffsetCommand extends CommandBase{
    @Override
    public void initialize(){
        TestSwerveModule.getInstance().configOffset();
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
