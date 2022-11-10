package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class SwerveDriveCommand extends CommandBase{

    // Joystick leftJoystick = new Joystick(1);
    // Joystick rightJoystick = new Joystick(0);

    double x = 0;
    double y = 0;
    double theta = 0;
    
    @Override
    public void initialize() {
        addRequirements(SwerveDriveSubsystem.getInstance());
        SmartDashboard.putNumber("MoveTest", 0);
        SmartDashboard.putNumber("TurnTest", 0);
    }

    @Override
    public void execute() {
        SwerveDriveSubsystem.getInstance().setTest(
            SmartDashboard.getNumber("MoveTest", 0), SmartDashboard.getNumber("TurnTest", 0)
            );

        SmartDashboard.putNumber("Angle1", SwerveDriveSubsystem.getInstance().getAngle1());
        SmartDashboard.putNumber("Angle2", SwerveDriveSubsystem.getInstance().getAngle2());
        SmartDashboard.putNumber("Angle3", SwerveDriveSubsystem.getInstance().getAngle3());
        SmartDashboard.putNumber("Angle4", SwerveDriveSubsystem.getInstance().getAngle4());
    }

    @Override
    public void end(boolean interrupted) {
        SwerveDriveSubsystem.getInstance().set(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
