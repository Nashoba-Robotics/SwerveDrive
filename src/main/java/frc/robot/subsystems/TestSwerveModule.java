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
import frc.robot.lib.util.Units;

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
        if(singleton == null)
         //singleton = new TestSwerveModule(Constants.SwerveBase.frontLeftMovePort,Constants.SwerveBase.frontLeftTurnPort, Constants.SwerveBase.frontLeftSensorPort, Constants.SwerveBase.frontLeftOffset);
         singleton = new TestSwerveModule(Constants.SwerveBase.frontRightMovePort,Constants.SwerveBase.frontRightTurnPort, Constants.SwerveBase.frontRightSensorPort, Constants.SwerveBase.frontLeftOffset);
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

    public double getMotorPos(){
        return turnMotor.getSelectedSensorPosition();
    }

    public double getTurnPos(){
        double motorPos = getMotorPos();
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

    //Takes input in degrees
    // Read in the current motor angle/pos & add/subtract needed NU to get to desired position
    public void testSet(double turn){
        double currentPos =  turnMotor.getSelectedSensorPosition();
        double lastTurn = NUToDeg(currentPos);

        double angle = fixedLowestAngle(turn, lastTurn);
        double angleChange = findAngleChange(angle, lastTurn);
        // if(Math.abs(angleChange) > 180){
        //     double dist = findDistance(turn, lastTurn);
        //     angleChange = Math.signum(turn-lastTurn)*dist;
        // } 

        double nextPos = currentPos + Units.degToNU(angleChange);
        turnMotor.set(ControlMode.MotionMagic, nextPos);
    }

    //Will probably annex into the findDistance function later
    public double findAngleChange(double turn, double lastTurn){
        double distance = turn - lastTurn;
        //double sign = Math.signum(distance);   //Either 1 or -1 -> represents positive or negative

        if(Math.abs(turn - (lastTurn + 360)) < Math.abs(distance)){
            // If this is true, it means that lastTurn is in the negatives and is trying to reach a positive, meaning that it must move positive
            distance = turn - (lastTurn + 360);
            //sign = +1;
        }

        if(Math.abs(turn+360 - (lastTurn)) < Math.abs(distance)){
            // If this is true, it means that turn is in the negatives and lastTurn is trying to reach a negative, meaning that you must move negative 
            distance = turn+360 - lastTurn;
            //sign = -1;
        }

        return distance;
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

    // TODO: Only use jank when it is necessary to get past a continuity error.
    //      Or check to see if it always works. Will there every be any sort of problem?
    public void jank(double turn, double lastTurn){
        double angle = fixedLowestAngle(turn, lastTurn);
        angle = degToNU(angle);

        turn = degToNU(turn);
        lastTurn = degToNU(lastTurn);

        //Reset the motor position so we will never see continuity issues
        turnMotor.setSelectedSensorPosition(0);

        turnMotor.set(ControlMode.MotionMagic, angle-lastTurn);

        //turnMotor.setSelectedSensorPosition(NUToDeg(angle));
        turnMotor.setSelectedSensorPosition(NUToDeg(turnSensor.getAbsolutePosition()));
        /*
            This method doesn't really account for error
            Maybe reading the angle from the CANcoder directly might be a bit better
            (Change later)
        */
    }

    public void jank(double turn){
        double lastTurn = NUToDeg(turnSensor.getAbsolutePosition());
        jank(turn, lastTurn);
    }

    public boolean isContinuityBreak(double turn, double lastTurn){
        if(Math.signum(turn) == Math.signum(lastTurn)) return false;
        if(Math.abs(turn) < 90 && Math.abs(lastTurn) < 90) return false;    //Not sure if this statement is correct
        return true;
    }
}
