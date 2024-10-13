// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  private final LaserCan lasercan = new LaserCan(60);
  private final CANdle candle = new CANdle(3,  "rio");

  /** Creates a new LEDs. */
  public LEDs() {
    configureLasercan();
    configureCANdle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void configureCANdle() {
    candle.configFactoryDefault();
    candle.configStatusLedState(false);
    candle.configLOSBehavior(false);
    candle.configV5Enabled(true);
    candle.configVBatOutput(VBatOutputMode.Off);
    candle.configBrightnessScalar(1.0);
    candle.configLEDType(LEDStripType.GRB);
  }

  private void configureLasercan() {
    try {
      lasercan.setRangingMode(RangingMode.SHORT);
      lasercan.setRegionOfInterest(new RegionOfInterest(4, 4, 8, 8));
      lasercan.setTimingBudget(TimingBudget.TIMING_BUDGET_100MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  public void setLED(int r, int g, int b, int start, int end) {
    int count = end - start + 1;
    candle.setLEDs(r, g, b, 0, start, count);
  }

  public double getLasercanMeasurement() {
    Measurement measurement = lasercan.getMeasurement();
    if(measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return measurement.distance_mm;
    }else{
      return -1.0;
    }
    //return -1.0;
  }
}
