// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FunnelConstants;

public class FunnelSubsystem extends SubsystemBase {
  /** Creates a new FunnelSubsystem. */
  public final WPI_VictorSPX FunnelRightMotor = new WPI_VictorSPX(FunnelConstants.kFunnelRightMotor);
<<<<<<< HEAD
  public final WPI_VictorSPX FunnelLeftMotor = new WPI_VictorSPX(FunnelConstants.kFunnelLeftMotor);
=======
  public final WPI_VictorSPX FunnelLeftMOtor = new WPI_VictorSPX(FunnelConstants.kFunnelLeftMotor);
>>>>>>> hooper
  public FunnelSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runFunnel(double speed, double _speed){
    FunnelRightMotor.set(speed);
<<<<<<< HEAD
    FunnelLeftMotor.set(_speed);
=======
    FunnelLeftMOtor.set(_speed);
>>>>>>> hooper
  }

  public void stopFunnel(){
    FunnelRightMotor.set(0);
<<<<<<< HEAD
    FunnelLeftMotor.set(0);
=======
    FunnelLeftMOtor.set(0);
>>>>>>> hooper
  }
}
