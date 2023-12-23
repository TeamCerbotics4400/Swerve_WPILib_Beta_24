// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team4400.Util;

import frc.robot.Constants.ModuleConstants;

/** Add your docs here. */
public class Conversions {
    public static double TalonFXRotationsToDistanceMeters(double rotations){
    return (rotations * (ModuleConstants.kWheelDiameterMeters * Math.PI)) 
                                                        / ModuleConstants.kDriveMotorGearRatio;
  }

  public static double meters2TalonFXRotations(double distanceInMeters){
    return (distanceInMeters / (Math.PI * ModuleConstants.kWheelDiameterMeters)) 
                                                        * ModuleConstants.kDriveMotorGearRatio;
  }

  public static double talonFXUnitsToRPM(double sensorUnits) {
    return (sensorUnits / ModuleConstants.CTRE_INTEGRATED_ENCODER_CPR) * 600.0;
  }
  
  public static double RPMtoTalonFXUnits(double RPM) {
    return (RPM / 600.0) * ModuleConstants.CTRE_INTEGRATED_ENCODER_CPR;
  }
}

