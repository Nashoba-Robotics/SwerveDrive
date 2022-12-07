package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import frc.robot.lib.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.util.SwerveState;

//
//NOTE: These are all in degrees -> Need to convert to radians
public class SwerveModule extends SubsystemBase{
    private TalonFX moveMotor, turnMotor;
    private CANCoder turnSensor;
    private double offset;
    private int moveMultiplier; //Will either be 1 or -1 to tell the move motor which direction it should go (Optimization purposes)

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

    //Takes in move and turn and sets module accordingly
    public void set(double move, double turn){
        double lastTurn = turnMotor.getSelectedSensorPosition(); //The current position is technically the lastPosition before it turns
        //Might be better to use CanCoder instead of integrated motor sensor

        if(isContinuityBreak(turn, lastTurn)){  //If there is a continuity break, we can use a special version of our set function to get around it
            breakContinuity(turn, lastTurn);
        }
        else{   //Otherwise, set it normally
            turnMotor.set(ControlMode.MotionMagic, Units.degToNU(findLowestAngle(turn, lastTurn)));
            
        }

        moveMotor.set(ControlMode.Velocity, move * moveMultiplier);
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
        if(Math.signum(turn) == Math.signum(lastTurn)) return false;
        if(Math.abs(turn) <= 90 && Math.abs(lastTurn) <= 90) return false;    //Not sure if this statement is correct
        return true;
    }

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