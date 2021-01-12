// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class JoystickConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }


    public static final class DriveConstants {

    }


        public static final class IntakeConstants{
            public static final int kIntakeMotorPort = 2;
            public static final int kCompressorPort = 1;
            public static final int kPCMPort = 0;
            public static final int kIntakeDoubleSolenoidPort = 2;
    

    }


    public static final class FunnelConstants{

    }

    public static final class AcceleratorConstants{
        
    }


    public static final class TurretConstants{

        public static final byte kTurretMotorPort = 0;
        public static final byte kTurretEncoderA = 0;
        public static final byte kTurretEncoderB = 0;
        public static final int kTurretEncoderPPR = 2048; //AMT-103
        public static final byte kToleranceInDegrees = 0;
        public static final byte kTurretHallEffect1Port = 0;
        public static final byte kTurretHallEffect2Port = 0;

        public static NeutralMode kTurretMotorMode = NeutralMode.Brake; //Brake-Coast

        public static final boolean kIsEncoderReversed = false;
        public static final boolean kIsMotorReversed = false;


        public static final double kP = 0.000;
        public static final double kI = 0.000;
        public static final double kD = 0.000;
        public static final double kS = 0.000;
        public static final double kV = 0.000;
        public static final double kA = 0.000;

        

    }


    public static final class ShooterConstants{
        
    }

    
    public static final class MiscConstants{

    }
}
