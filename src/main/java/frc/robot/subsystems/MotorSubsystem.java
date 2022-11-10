package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MotorSubsystem extends SubsystemBase {

    private static int i = 0;
    
    private static MotorSubsystem instance;
    
    private static TalonFX motor1, motor2, motor3, motor4;
    
    public static MotorSubsystem getInstance() {
        if (instance == null) instance = new MotorSubsystem();
        return instance;
    }
    
    private MotorSubsystem() {
        motor1 = createMotor();    
        motor2 = createMotor();    
        motor3 = createMotor();    
        motor4 = createMotor();    
    }

    public void setMotor(double val) {
        motor1.set(ControlMode.PercentOutput, val);
        motor2.set(ControlMode.PercentOutput, val);
        motor3.set(ControlMode.PercentOutput, val);
        motor4.set(ControlMode.PercentOutput, val);
    }

    private TalonFX createMotor() {
        TalonFX motor = new TalonFX(i++);
        configureMotor(motor);
        return motor;
    }
    
    private void configureMotor(TalonFX motor) {
        motor.configFactoryDefault();
		
		/* Config neutral deadband to be the smallest possible */
		motor.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
        motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            0, 
											0);
                                                
		/* Config the peak and nominal outputs */
		motor.configNominalOutputForward(0, 0);
		motor.configNominalOutputReverse(0, 0);
		motor.configPeakOutputForward(1, 0);
		motor.configPeakOutputReverse(-1, 0);

        motor.selectProfileSlot(0, 0);

        motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_1Ms, 30);
        motor.configVelocityMeasurementWindow(8, 30);

        motor.setNeutralMode(NeutralMode.Coast);

        motor.setSelectedSensorPosition(0, 0, 0);
        motor.clearStickyFaults();
    }
}
