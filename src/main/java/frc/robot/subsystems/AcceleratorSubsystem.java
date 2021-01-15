// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AcceleratorConstants;

public class AcceleratorSubsystem extends SubsystemBase {
  /** Creates a new AcceleratorSubsystem. */
  private final WPI_VictorSPX AccelaratorMotor = new WPI_VictorSPX(AcceleratorConstants.kAccelaratorMotor);
  public AcceleratorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runAccelarator(double speed){
    AccelaratorMotor.set(speed);
  }

  public void stopAccelarator(){
    AccelaratorMotor.set(0);
  }
}
