package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.lib.SwerveState;

public class SwerveModule {

    private TalonFX moveMotor;
    private TalonFX turnMotor;

    public SwerveModule(int movePort, int turnPort) {
        moveMotor = new TalonFX(movePort);
        turnMotor = new TalonFX(turnPort);
    }
    
    public void set(SwerveState state){}

}
