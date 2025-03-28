// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class limelight extends SubsystemBase {
  private NetworkTable table;
    private int closestID;
    public double x;
    public double z;
    public double yaw;
    private NetworkTableEntry tid;
    private NetworkTableEntry tg;
    private double[] cpose;
    public DoubleSupplier X=()->0;
    public DoubleSupplier Y=()->0;
    public DoubleSupplier YAW;
  /** Creates a new ExampleSubsystem. */
  public limelight() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
    public int getID(){
      return closestID;
  }
    public void updateLimelight(){
      table = NetworkTableInstance.getDefault().getTable("limelight");
      tid = table.getEntry("tid");
      tg = table.getEntry("camerapose_targetspace");
      closestID = (int) tid.getInteger(0);
      cpose = tg.getDoubleArray(new double[]{4688, 4688, -4688, 4688, 4688, 4688, 4688});
      x = cpose[0];
      z = -cpose[2];
      yaw = cpose[4];
      YAW = ()-> yaw;

      SmartDashboard.putNumber("Closest ID",closestID);
      SmartDashboard.putNumber("targetX",x);
      SmartDashboard.putNumber("targetZ",z);
      SmartDashboard.putNumber("target Yaw", yaw);
  }

  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    updateLimelight();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
