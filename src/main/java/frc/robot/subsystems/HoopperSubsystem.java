// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HooperConstants;

public class HoopperSubsystem extends SubsystemBase {
  /** Creates a new HooperSubsystem. */
  public final Victor HoopperLeftMotor = new Victor(HooperConstants.HoopperLeftMotor);
  public final Victor HoopperRightMotor = new Victor(HooperConstants.HoopperRightMotor);
  public HoopperSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runHooper(Double speed){
    HoopperLeftMotor.set(speed);
    HoopperRightMotor.set(speed);
  }

  public void stopHopper(){
    HoopperLeftMotor.set(0);
    HoopperRightMotor.set(0);
  }
}
