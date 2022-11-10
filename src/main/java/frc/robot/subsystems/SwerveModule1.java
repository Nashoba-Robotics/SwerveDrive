package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.util.SwerveState;

public class SwerveModule1 {

    private TalonFX moveMotor;
    private TalonFX turnMotor;

    private CANCoder turnSensor;

    public SwerveModule1(int movePort, int turnPort, int sensorPort) {
        moveMotor = new TalonFX(movePort);
        turnMotor = new TalonFX(turnPort);

        // moveMotor.configRemoteFeedbackFilter(moveSensor, 0, 0);
        // moveMotor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 0);
        
        turnSensor = new CANCoder(sensorPort);
        turnSensor.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        turnSensor.configMagnetOffset(-88);
        //turnMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        

        //Temporary values
        turnMotor.configFeedbackNotContinuous(true, 0);
        turnMotor.config_kF(0, 0.04);
        turnMotor.config_kP(0, 0.025);
        //turnMotor.config_kI(0, 0.0025);
        turnMotor.configMotionCruiseVelocity(10_000);
        turnMotor.configMotionAcceleration(20_000);

        turnMotor.setSelectedSensorPosition(turnSensor.getAbsolutePosition()/360 * 150/7 * 2048);
    }
    
    //NEGATIVES ARE WEIRD
    //  Ex. -3.14 -> 0 = spins a lot    Also, 0 -> -1  == 0 -> 1
    //Assuming state.turn is within constraints
    public void set(SwerveState state){
        //Turning swerve modules:
        //When we want to turn to a specific angle, there are two possible locations for our wheel to go
        //  We can go to the desired location
        //  or we could go to the opposite and send the speed in the opposite direction
        // We want to figure out which one is closer and go to that one
        double turn = constrain(state.turn);
        //SmartDashboard.putNumber("? Turn", turn);
        double lastTurn = constrain(state.lastTurn);

        double oppositeTurn = constrain((turn + Math.PI));
        //SmartDashboard.putNumber("? OppTurn", oppositeTurn);
        //Find distances between the two to compare
        double distance = Math.abs(lastTurn-turn);
        double oppositeDistance = Math.abs(lastTurn-oppositeTurn);
        SmartDashboard.putNumber("Distance", distance);
        SmartDashboard.putNumber("Opp Dist", oppositeDistance);
        
        int moveMultiplier = 1; //If we do go to the opposite angle, we will need to move in the opposite direction than originally intended
        double finalValue;
        if(distance < oppositeDistance) finalValue = turn;
        else{
            finalValue = oppositeTurn;
            moveMultiplier = -1;
        } 

        //TODO: Account for gear ratio - 6.21: 1 (moving)   150/7 : 1 (turning)
        // double turnValue = ((finalValue-lastTurn) * (2048/(2*Math.PI))); //Fullrotation = 2048 (According to Ben)
        double turnValue = (finalValue * (2048/(2*Math.PI)));
        turnValue *= 150.0/7; //Accounting for gear ratio
        //turnMotor.configMotionCruiseVelocity(15000000);
        
        //turnMotor.setSelectedSensorPosition(0); //Zero the motor
        turnMotor.set(ControlMode.MotionMagic, turnValue);  //then move the motor according to how it should

        moveMotor.set(ControlMode.PercentOutput, state.move * moveMultiplier);
    }

    public void dumbSet(SwerveState state){
        double turn = constrain(state.turn);
        double turnValue = ((turn) * (2048/(2*Math.PI)));; //convert from radians to native units
        //150/7 : 1
        turnValue *= 150./7;

        turnMotor.set(ControlMode.MotionMagic, turnValue);  //then move the motor according to how it should

        moveMotor.set(ControlMode.PercentOutput, state.move);
    }

    public void veryDumbSet(double turn){
        turnMotor.set(ControlMode.MotionMagic, turn);
    }

    //Constrains input into -tau/2 - tau/2
    public double constrain(double angle){
        angle %= (2*Math.PI);
        if(Math.abs(angle) < Math.PI){
            return angle;
        }

        angle = Math.abs(angle) - 2*Math.PI; 
        return angle == -Math.PI ? 0 : angle;
    }
    
    // public void reset(){
    //     turnSensor.setPosition(0);
    //     turnMotor.setSelectedSensorPosition(0);
    // }
    
    public double getTurnPosition(){
        return turnMotor.getSelectedSensorPosition();
    }

    public double getTrueTurnPosition(){
        return turnSensor.getAbsolutePosition();
    }

    public double NU2Radians(double NU){
        return NU * 2.0*Math.PI/2048;
    }
}
