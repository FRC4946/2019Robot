/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * Add your docs here.
 */
public class PIDSourceGyro implements PIDSource {
    
    AHRS m_gyro;  
    PIDSourceType m_PIDSourceType;
  
    public PIDSourceGyro(AHRS gyro) {
      m_gyro = gyro;
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
      return m_gyro.getYaw();
    }
  }
