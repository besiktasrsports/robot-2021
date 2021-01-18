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
        public static final int kOperatorControllerPort = 0;
    }


    public static final class DriveConstants {
        public static final int kLeftRearMotor = 10;
        public static final int kLeftFrontMotor = 11;
        public static final int kRightRearMotor = 12;
        public static final int kRightFrontMotor = 13;
        public static final boolean kLeftRearMotorInverted = false;
        public static final boolean kLeftFrontMotorInverted = false;
        public static final boolean kRightRearMotorInverted = false;
        public static final boolean kRightFrontMotorInverted = false;
    }


    public static final class IntakeConstants{
        public static final int kIntakeMotorPort = 40;
        public static final int kCompressorPort = 0;
        public static final int kPCMPort = 0;
        public static final int kIntakeDoubleSolenoidPort1 = 0;
        public static final int kIntakeDoubleSolenoidPort2 = 1;


    }

    public static final class FunnelConstants{

        public static final int kFunnelRightMotor = 20;
        public static final int kFunnelLeftMotor = 21;
    }

    public static final class AcceleratorConstants{
        
    }


    public static final class TurretConstants{

        public static final byte kTurretMotorPort = 0;
        public static final byte kTurretEncoderA = 2;
        public static final byte kTurretEncoderB = 3;
        public static final int kTurretEncoderPPR = 2048; //AMT-103
        public static final byte kToleranceInDegrees = 0;
        public static final byte kTurretHallEffect1Port = 0;
        public static final byte kTurretHallEffect2Port = 1;

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
        public static final int kShooterMotor1Port = 30;
        public static final int kShooterMotor2Port = 31;
        public static final boolean kShooterInvertedMode1 = true;
        public static final boolean kShooterInvertedMode2 = true;

    }

    
    public static final class MiscConstants{

    }
}
