// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsytem extends SubsystemBase {
  /** Creates a new DriveTrain. */

  private final VictorSPX m_motor = new VictorSPX(0); 
  public DriveSubsytem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
