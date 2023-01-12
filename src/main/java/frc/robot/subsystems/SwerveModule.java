package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.lib.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.util.SwerveState;

//Make algorithm to lock into increments of 90

//
//NOTE: These are all in degrees -> Need to convert to radians
public class SwerveModule extends SubsystemBase{
    private TalonFX moveMotor, turnMotor;
    private CANCoder turnSensor;
    private double offset;
    private int moveMultiplier; //Will either be 1 or -1 to tell the move motor which direction it should go (Optimization purposes)
    //I don't think move multiplier should be global
    public SwerveModule(int movePort, int turnPort, int sensorPort, double offset){
        moveMotor = new TalonFX(movePort);
        turnMotor = new TalonFX(turnPort);

        turnSensor = new CANCoder(sensorPort);
        this.offset = offset;

        moveMultiplier = 1;
        
        //Testing
        turnMotor.setInverted(InvertType.InvertMotorOutput);
    }

    public void zero(){
        resetMotorPos();
        set(0, 0);
    }

    public void resetMotorPos(){
        turnSensor.configMagnetOffset(-offset);
        turnMotor.setSelectedSensorPosition(Units.degToNU(turnSensor.getAbsolutePosition()));
    }

    public double getMotorPos(){
        return turnMotor.getSelectedSensorPosition();
    }

    public double getSensorPos(){
        return turnSensor.getAbsolutePosition();
    }

    // Takes in a SwerveState(move and turn) and moves the module correspondingly
    public void set(SwerveState state){
        set(state.move, state.turn);
    }

    public SwerveModuleState getState(){
        //Speed(mps), angle (Rotation2D(degrees))
        SwerveModuleState state = new SwerveModuleState(
            toMPS(moveMotor.getSelectedSensorVelocity()),
            new Rotation2d(Units.NUToDeg(turnMotor.getSelectedSensorPosition()) * Math.PI/180)   //radians
        );
        return state;
    }

    //Converts from NU/100ms
    public double toMPS(double speed){
        double wheelCircumference = 0;
        speed = speed/4096*wheelCircumference;
        speed /= 10;
        return speed;

    }

    //Takes in move and turn and sets module accordingly
    // public void set(double move, double turn){
    //     double lastTurn = Units.NUToDeg(turnMotor.getSelectedSensorPosition()); //The current position is technically the lastPosition before it turns
    //     //Might be better to use CanCoder instead of integrated motor sensor

    //     if(isContinuityBreak(turn, lastTurn)){  //If there is a continuity break, we can use a special version of our set function to get around it
    //         breakContinuity(turn, lastTurn);
    //     }
    //     else{   //Otherwise, set it normally
    //         turnMotor.set(ControlMode.MotionMagic, Units.degToNU(findLowestAngle(turn, lastTurn)));
    //     }

    //     moveMotor.set(ControlMode.Velocity, move * moveMultiplier);
    // }

    public void set(double move, double turn){
        double currentPos =  turnMotor.getSelectedSensorPosition();
        double lastTurn = Units.NUToDeg(currentPos);

        double angle = findLowestAngle(turn, lastTurn);
        double angleChange = findAngleChange(angle, lastTurn);
        // if(Math.abs(angleChange) > 180){
        //     double dist = findDistance(turn, lastTurn);
        //     angleChange = Math.signum(turn-lastTurn)*dist;
        // } 
        
        double nextPos = currentPos + Units.degToNU(angleChange);

        turnMotor.set(ControlMode.MotionMagic, nextPos);
    }

    /*
    Optimization to find the actual angle we want to get to:
        When setting the angle, we can either go to that specified angle,
        or we could go to the opposite of that angle and reverse the direction of movement
            (This is where move multiplier comes in)
    */
    public double findLowestAngle(double turn, double lastTurn){
        double[] potAngles = potentialAngles(turn); //Gets the two potential angles we could go to

        // Calculate the distance between those and the last angle the module was at
        double originalDistance = findDistance(potAngles[0], lastTurn);
        double oppositeDistance = findDistance(potAngles[1], lastTurn);

        // If the original distance is less, we want to go there
        if(originalDistance <= oppositeDistance){
            moveMultiplier = 1;
            return potAngles[0];
        }
        else{
            moveMultiplier = -1;
            return potAngles[1];
        } 

        /*
         * double[]potAngles = potentialAngles(turn);

        double originalDistance = findDistance(potAngles[0], lastTurn);

        double oppositeDistance = findDistance(potAngles[1], lastTurn);

        double desiredAngle = 0;
        if(originalDistance <= oppositeDistance){
            desiredAngle = potAngles[0];
        }
        else desiredAngle = potAngles[1];

        return desiredAngle;
         */
    }

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

    // Find the two angles we could potentially go to
    public double[] potentialAngles(double angle){
        //Constrain the variable to desired domain
        angle = Units.constrainDeg(angle);
        //Figure out the opposite angle
        double oppositeAngle = angle + 180;
        //Constrain the opposite angle
        oppositeAngle = Units.constrainDeg(oppositeAngle);
        //Put them into a size 2 array
        double[] angles = {angle, oppositeAngle};
        //return it
        return angles;
    }

    //Finds the distance between the angle we want to get to and the previous angle
    double findDistance(double turn, double lastTurn){
        double distance = Math.min(Math.abs(turn - lastTurn), Math.abs(turn+360 - lastTurn));
        distance = Math.min(distance, Math.abs(turn - (lastTurn+360)));

        //Ex. 150 -> -150 should have a distance of 60 since we will break continuity
        //      However, standard formula of finding distance (|last-current|) will give a wrong answer

        return distance;
    }

    

    public boolean isContinuityBreak(double turn, double lastTurn){
        if(Math.signum(turn) == Math.signum(lastTurn) || turn == 0 || lastTurn == 0) return false;
        if(Math.abs(turn) <= 90 && Math.abs(lastTurn) <= 90) return false;    //Not sure if this statement is correct
        return true;
    }

    //I don't think the motor is resetting correctly
    // A method to get past continuity breaks
    public void breakContinuity(double turn, double lastTurn){
        /*
         * We essentially reset the motor's sensor to make it think its at 0,
         * move it the correct distance,
         * and then set it to what it's supposed to be at
         */

        double angle = findLowestAngle(turn, lastTurn);
        angle = Units.degToNU(angle);

        turn = Units.degToNU(turn);
        lastTurn = Units.degToNU(lastTurn);

        //Reset the motor position so we will never see continuity issues
        turnMotor.setSelectedSensorPosition(0);

        turnMotor.set(ControlMode.MotionMagic, angle-lastTurn);

        //turnMotor.setSelectedSensorPosition(NUToDeg(angle));
        turnMotor.setSelectedSensorPosition(Units.NUToDeg(turnSensor.getAbsolutePosition()));
        /*
            This method doesn't really account for error
            Maybe reading the angle from the CANcoder directly might be a bit better
            (Change later)
        */
    }

}
