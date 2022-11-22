package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.util.SwerveState;

/*
    TODO: Change name of jank
        Reorganize
        Comment
*/


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
    }

    public void set(SwerveState state){
        set(state.move, state.turn);
    }

    public void set(double move, double turn){
        double lastTurn = turnMotor.getSelectedSensorPosition(); //The current position is technically the lastPosition before it turns
        //Might be better to use CanCoder instead of integrated motor sensor

        if(isContinuityBreak(turn, lastTurn)){
            jank(turn, lastTurn);
        }
        else{
            turnMotor.set(ControlMode.MotionMagic, degToNU(findLowestAngle(turn, lastTurn)));
        }

        moveMotor.set(ControlMode.Velocity, move * moveMultiplier);
    }

    //Optimization to find the actual angle we want to get to
    public double findLowestAngle(double turn, double lastTurn){
        double[] potAngles = potentialAngles(turn);

        double originalDistance = findDistance(potAngles[0], lastTurn);
        double oppositeDistance = findDistance(potAngles[1], lastTurn);

        if(originalDistance <= oppositeDistance){
            moveMultiplier = 1;
            return potAngles[0];
        }
        else{
            moveMultiplier = -1;
            return potAngles[1];
        } 
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

    public boolean isContinuityBreak(double turn, double lastTurn){
        if(Math.signum(turn) == Math.signum(lastTurn)) return false;
        if(Math.abs(turn) < 90 && Math.abs(lastTurn) < 90) return false;    //Not sure if this statement is correct
        return true;
    }

    public void jank(double turn, double lastTurn){
        double angle = findLowestAngle(turn, lastTurn);
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

}
