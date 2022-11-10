package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.util.SwerveState;
import frc.robot.subsystems.TestSwerveModule;

public class SwerveDriveSubsystem extends SubsystemBase {
    
    // private TestSwerveModule module1;
    // private TestSwerveModule module2;
    // private TestSwerveModule module3;
    // private TestSwerveModule module4;
    private SwerveModule mod1;
    private SwerveModule mod2;
    private SwerveModule mod3;
    private SwerveModule mod4;



    private Pigeon2 gyro;

    private SwerveDriveSubsystem() {
        
        mod1 = Mk4iSwerveModuleHelper.createFalcon500(
            GearRatio.L3, Constants.SwerveBase.frontRightMovePort,
            Constants.SwerveBase.frontRightTurnPort, Constants.SwerveBase.frontRightSensorPort,
            Constants.SwerveBase.frontRightOffsetRadians
            );        

        mod2 = Mk4iSwerveModuleHelper.createFalcon500(
            GearRatio.L3, Constants.SwerveBase.frontLeftMovePort,
            Constants.SwerveBase.frontLeftTurnPort, Constants.SwerveBase.frontLeftSensorPort,
            Constants.SwerveBase.frontLeftOffsetRadians
            );

        mod3 = Mk4iSwerveModuleHelper.createFalcon500(
            GearRatio.L3, Constants.SwerveBase.backLeftMovePort,
            Constants.SwerveBase.backLeftTurnPort, Constants.SwerveBase.backLeftSensorPort,
            Constants.SwerveBase.backLeftOffsetRadians
            );

        mod4 = Mk4iSwerveModuleHelper.createFalcon500(
            GearRatio.L3, Constants.SwerveBase.backRightMovePort,
            Constants.SwerveBase.backRightTurnPort, Constants.SwerveBase.backRightSensorPort,
            Constants.SwerveBase.backRightOffsetRadians
            );

        // module1 = new TestSwerveModule(
        //     Constants.SwerveBase.frontRightMovePort, 
        //     Constants.SwerveBase.frontRightTurnPort, 
        //     Constants.SwerveBase.frontRightSensorPort, 
        //     Constants.SwerveBase.frontRightOffset
        // );
        // module2 = new TestSwerveModule(
        //     Constants.SwerveBase.frontLeftMovePort, 
        //     Constants.SwerveBase.frontLeftTurnPort, 
        //     Constants.SwerveBase.frontLeftSensorPort, 
        //     Constants.SwerveBase.frontLeftOffset
        // );
        // module3 = new TestSwerveModule(
        //     Constants.SwerveBase.backLeftMovePort, 
        //     Constants.SwerveBase.backLeftTurnPort, 
        //     Constants.SwerveBase.backLeftSensorPort, 
        //     Constants.SwerveBase.backLeftOffset
        // );
        // module4 = new TestSwerveModule(
        //     Constants.SwerveBase.backRightMovePort, 
        //     Constants.SwerveBase.backRightTurnPort, 
        //     Constants.SwerveBase.backRightSensorPort, 
        //     Constants.SwerveBase.backRightOffset
        // );

        gyro = new Pigeon2(0);
    }
    
    private static SwerveDriveSubsystem instance;

    public static SwerveDriveSubsystem getInstance() {
        if(instance == null) {
            instance = new SwerveDriveSubsystem();
        }
        return instance;
    }

    private double getGyroAngle() {
        return gyro.getYaw() * Math.PI / 180;
    }

    public void set(double x, double y, double omega) {

        double r = Math.sqrt(x*x + y*y);
        double tempAngle = Math.atan2(y, x);

        // x = r * Math.cos(tempAngle + getGyroAngle());
        // y = r * Math.sin(tempAngle + getGyroAngle());

        x = r * Math.cos(tempAngle + 0);
        y = r * Math.sin(tempAngle + 0);

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
            Math.atan2(y1, x1),
            0
            );

        SwerveState s2 = new SwerveState(
            Math.sqrt(x2*x2 + y2*y2),
            Math.atan2(y2, x2),
            0
        );

        SwerveState s3 = new SwerveState(
            Math.sqrt(x3*x3 + y3*y3),
            Math.atan2(y3, x3),
            0
        );

        SwerveState s4 = new SwerveState(
            Math.sqrt(x4*x4 + y4*y3),
            Math.atan2(y4, x4),
            0
        );

        // module1.set(s1);
        // module2.set(s2);
        // module3.set(s3);
        // module4.set(s4);

        mod1.set(12 * s1.move, s1.turn);
        mod2.set(12 * s2.move, s2.turn);
        mod3.set(12 * s3.move, s3.turn);
        mod4.set(12 * s4.move, s4.turn);

    }

    public void setTest(double move, double turn) {
        mod1.set(12 * move, turn);
        mod2.set(12 * move, turn);
        mod3.set(12 * move, turn);
        mod4.set(12 * move, turn);
    }

    public double getAngle1() {
        return mod1.getSteerAngle();
    }

    public double getAngle2() {
        return mod2.getSteerAngle();
    }

    public double getAngle3() {
        return mod3.getSteerAngle();
    }

    public double getAngle4() {
        return mod4.getSteerAngle();
    }
}