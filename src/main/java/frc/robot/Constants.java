// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class SwerveBase {
        //Approximate
        public static final double width = .548;    //meters
        public static final double length = .548;
        public static final double hDiagonal = Math.sqrt(width*width + length*length)/2;

        public static final int frontRightTurnPort = 0;
        public static final int frontLeftTurnPort = 1;
        public static final int backLeftTurnPort = 2;
        public static final int backRightTurnPort = 3;

        public static final int frontRightMovePort = 4;
        public static final int frontLeftMovePort = 5;
        public static final int backLeftMovePort = 6;
        public static final int backRightMovePort = 7;

        public static final int frontRightSensorPort = 0;
        public static final int frontLeftSensorPort = 1;
        public static final int backLeftSensorPort = 2;
        public static final int backRightSensorPort = 3;
        
        public static final double frontRightOffset = -75.63;
        public static final double frontLeftOffset = -43.066;
        public static final double backLeftOffset = -121.318;
        public static final double backRightOffset = 96.68;

        public static final double frontRightOffsetRadians = -105.36 * Math.PI / 180;
        public static final double frontLeftOffsetRadians = 0;
        public static final double backLeftOffsetRadians = 0;
        public static final double backRightOffsetRadians = 0;

    }

}
