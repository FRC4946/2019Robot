/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Add your docs here.
 */
public class PIDSourceEnc implements PIDSource {

  CANEncoder m_encoder; 
  PIDSourceType m_PIDSourceType;

  public PIDSourceEnc(CANEncoder enc) {
    m_encoder = enc;
    m_PIDSourceType = PIDSourceType.kDisplacement;
  }

  @Override
  public PIDSourceType getPIDSourceType() {
    return m_PIDSourceType; 
  }

  @Override
  public void setPIDSourceType(PIDSourceType pidSourceType) {
    m_PIDSourceType = pidSourceType;
  }

  @Override
  public double pidGet() {
    return m_encoder.getPosition();
  }
}
