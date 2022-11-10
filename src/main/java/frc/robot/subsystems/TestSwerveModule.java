package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.lib.util.SwerveState;

public class TestSwerveModule {
    TalonFX turnMotor;
    TalonFX moveMotor;
    CANCoder turnSensor;
    double offSet;

    double moveMultiplier;

    public TestSwerveModule(int movePort, int turnPort, int sensorPort, double offSet){
        this.offSet = offSet;
        turnMotor = new TalonFX(turnPort);
        turnSensor = new CANCoder(sensorPort);
        moveMotor = new TalonFX(movePort);

        moveMultiplier = 1;

        turnMotor.configNeutralDeadband(0.001);
        turnMotor.config_kF(0, 0.0475);
        turnMotor.config_kP(0, 0.2); //0.0475  (current) 0.05
        //turnMotor.config_kI(0, 0.0025);
        turnMotor.config_kD(0, 0.1);

        turnMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 45, 0.2));
        turnMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 65, 0.2));

        int cruiseVelocity = 20_000;
        turnMotor.configMotionCruiseVelocity(cruiseVelocity);
        turnMotor.configMotionAcceleration(2*cruiseVelocity);

        
        turnMotor.configFeedbackNotContinuous(true, 0);
        // turnSensor.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        // turnSensor.configMagnetOffset(-75.63);
        // turnMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        // turnMotor.setSelectedSensorPosition(turnSensor.getAbsolutePosition()/360 * 150/7 * 2048);
        configOffset();
    }

    private static TestSwerveModule singleton;
    public static TestSwerveModule getInstance(){
        if(singleton == null) singleton = new TestSwerveModule(4,Constants.SwerveBase.frontRightTurnPort, Constants.SwerveBase.frontRightSensorPort, Constants.SwerveBase.frontRightOffset);
        return singleton;
    }

    public double constrainDeg(double angle){
        angle %= 360;

        if(Math.abs(angle) <= 180){
            return angle;
        }

        if(angle > 0){
            return angle - 360;
        }
        else{
            return angle + 360;
        }
    }

    public double getTurnPos(){
        double motorPos = turnMotor.getSelectedSensorPosition();
        return Math.signum(motorPos)*Math.abs(motorPos)%(2048.*150/7);
    }

    public double getAbsPos(){
        return turnSensor.getAbsolutePosition();
    }

    public void toZero(){
        turnMotor.set(ControlMode.MotionMagic, 0);
    }

    //In degrees
    public void setPos(double turn, double lastTurn){
        turn = degToNU(findLowestAngle(turn, lastTurn));
        turnMotor.set(ControlMode.Position, turn);
    }

    //Setting degrees to 190 seems to put it at -170, but it doesn't work with optimize set 11/04/22
    public void dumbSet(double turn){
        turn = degToNU(turn);
        turnMotor.set(ControlMode.MotionMagic, turn);
    }

    //In degrees
    public void set(double turn){
        turn = turn / 360. * 2048;
        turn *= 150./7;
        turnMotor.set(ControlMode.MotionMagic, turn);
    }

    public void set(SwerveState state){
        double move = state.move;
        double turn = state.turn * 360 / (2*Math.PI);
        double lastTurn = state.lastTurn * 360 / (2*Math.PI);

        optimizeSet(move, turn, lastTurn);
    }

    //In degrees
    public void optimizeSet(double move, double turn, double lastTurn){
        turnMotor.set(ControlMode.MotionMagic, degToNU(findLowestAngle(turn, lastTurn)));
        moveMotor.set(ControlMode.PercentOutput, move * moveMultiplier);
    }

    public void testSet(double turn, double lastTurn){
        turnMotor.set(ControlMode.MotionMagic, degToNU(findLowestAngle(turn, lastTurn)));
    }

    //TODO: lastTurn = 170 & turn = -170
    //      Make it turn to 190 instead of -170
    //Returns the closest of the angle we want and the opposite angle
    public double findLowestAngle(double turn, double lastTurn){
        double[] potAngles = potentialAngles(turn);
        //potAngles[0] = original angle
        //potAngles[1] = opposite angle

        // double originalDistance = Math.abs(potAngles[0] - lastTurn);
        double originalDistance = findDistance(potAngles[0], lastTurn);
        //double oppositeDistance = Math.abs(potAngles[1] - lastTurn);
        double oppositeDistance = findDistance(potAngles[1], lastTurn);

        if(originalDistance <= oppositeDistance){
            // go to original angle
            moveMultiplier = 1;
            return potAngles[0];
        }
        else{
            // go to opposite angle
            moveMultiplier = -1;
            return potAngles[1];
        }
    }

    //For Temporary Testing
    public double fixedLowestAngle(double turn, double lastTurn){
        double[]potAngles = potentialAngles(turn);

        double originalDistance = findDistance(potAngles[0], lastTurn);

        double oppositeDistance = findDistance(potAngles[1], lastTurn);

        double desiredAngle = 0;
        if(originalDistance <= oppositeDistance){
            desiredAngle = potAngles[0];
        }
        else desiredAngle = potAngles[1];

        double distance = Math.min(originalDistance, oppositeDistance);
        if(Math.signum(turn) != Math.signum(lastTurn)){
            if(Math.abs(turn) > 90 && Math.abs(lastTurn) > 90){
                desiredAngle = lastTurn + Math.signum(lastTurn)*distance;
            }
        }
        return desiredAngle;
    }

    double findDistance(double turn, double lastTurn){
        double distance = Math.min(Math.abs(turn - lastTurn), Math.abs(turn+360 - lastTurn));
        distance = Math.min(distance, Math.abs(turn - (lastTurn+360)));

        return distance;
    }

    public double degToNU(double angle){
        //Convert from degrees into rotation
        angle /= 360;

        //Convert from rotations into Native Units
        angle *= 2048;

        //Account for gear ratio
        angle *= 150./7;

        return angle;
    }

    public double NUToDeg(double angle){
        angle /= 150./7;
        angle /= 2048;
        angle *= 360;

        return angle;
    }

    public double[] potentialAngles(double angle){
        //Constrain the variable to desired domain
        angle = constrainDeg(angle);
        //Figure out the opposite angle
        double oppositeAngle = angle + 180;
        //Constrain it
        oppositeAngle = constrainDeg(oppositeAngle);
        //Put them into a size 2 array
        double[] angles = {angle, oppositeAngle};
        //return it
        return angles;
    }

    public void stop(){
        turnMotor.set(ControlMode.PercentOutput, 0);
    }

    public void configOffset(){
        turnSensor.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
        //turnSensor.configFeedbackCoefficient(sensorCoefficient, unitString, sensorTimeBase)
        
        turnSensor.configMagnetOffset(offSet);
        SmartDashboard.putNumber("absolute angle", turnSensor.getAbsolutePosition());
        turnMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        turnMotor.setSelectedSensorPosition(turnSensor.getAbsolutePosition()/360 * 150/7 * 2048);
        turnMotor.setInverted(InvertType.InvertMotorOutput);
    }

    //For Testing
    //What happens if you change the selectedSensorPosition after setting the angle with motion magic
    public void jankContinuity(double turn, double lastTurn){
        //double lastAngle = turnMotor.getSelectedSensorPosition();
        turn = degToNU(turn);
        lastTurn = degToNU(lastTurn);
        turnMotor.setSelectedSensorPosition(0);
        turnMotor.set(ControlMode.MotionMagic, lastTurn-turn);
        turnMotor.setSelectedSensorPosition(NUToDeg(turn));
    }

    //Hopefully this works...
    //It didn't, but it seems like it might
    public void jank(double turn, double lastTurn){
        double angle = findLowestAngle(turn, lastTurn);
        angle = degToNU(angle);

        lastTurn = degToNU(lastTurn);

        //Reset the motor position so we will never see continuity issues
        turnMotor.setSelectedSensorPosition(0);

        turnMotor.set(ControlMode.MotionMagic, angle);

        turnMotor.setSelectedSensorPosition(NUToDeg(angle));
        /*
            This method doesn't really account for error
            Maybe reading the angle from the CANcoder directly might be a bit better
            (Change later)
        */
    }
}
