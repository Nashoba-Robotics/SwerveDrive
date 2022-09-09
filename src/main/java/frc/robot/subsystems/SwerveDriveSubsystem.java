package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.SwerveState;

public class SwerveDriveSubsystem extends SubsystemBase {
    
    private SwerveModule module1;
    private SwerveModule module2;
    private SwerveModule module3;
    private SwerveModule module4;

    private SwerveDriveSubsystem() {
        module1 = new SwerveModule(0, 0);
        module2 = new SwerveModule(0, 0);
        module3 = new SwerveModule(0, 0);
        module4 = new SwerveModule(0, 0);
    }
    
    private SwerveDriveSubsystem instance;

    public SwerveDriveSubsystem getInstance() {
        if(instance == null) {
            instance = new SwerveDriveSubsystem();
        }
        return instance;
    }

    public void set(double x, double y, double omega) {

        //Repeated equations
        double h = omega * Constants.SwerveBase.width/2;
        double b = omega * Constants.SwerveBase.length/2;

        //The addition of the movement and rotational vector
        double x1 = x - b;
        double y1 = y + h;

        double x2 = x - b;
        double y2 = y - h;

        double x3 = x + b;
        double y3 = y - h;

        double x4 = x + b;
        double y4 = y + h;

        //Convert to polar
        SwerveState s1 = new SwerveState(
            Math.sqrt(x1*x1 + y1*y1),
            Math.atan2(y1, x1)
            );

        SwerveState s2 = new SwerveState(
            Math.sqrt(x2*x2 + y2*y2),
            Math.atan2(y2, x2)
        );

        SwerveState s3 = new SwerveState(
            Math.sqrt(x3*x3 + y3*y3),
            Math.atan2(y3, x3)
        );

        SwerveState s4 = new SwerveState(
            Math.sqrt(x4*x4 + y4*y3),
            Math.atan2(y4, x4)
        );

        module1.set(s1);
        module2.set(s2);
        module3.set(s3);
        module4.set(s4);

    }
}