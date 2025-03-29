// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Arm extends SubsystemBase {
  SparkMax elevator1;
  SparkMax elevator2;
  double espeed;
  SparkMax intake1;
  SparkMax intake2;
  public SparkMax coral;
  SparkMax climb;
  DigitalInput stopClimb;

  double l1 = 8.3;
  double l12 = -8.3;
  double l2 = 29.8;
  double l22 = -29.8;
  double l3 = 65.3;
  double l32 = -65.3;
  double l4;
  double l42;
  double a1 = 31.1;
  double a12 = -31.1;
  double a2 = 54.1;
  double a22 = -54.1;

  /* Creates a new ExampleSubsystem. */
  public Arm() {
    elevator1 = new SparkMax(26, MotorType.kBrushless);
    elevator2 = new SparkMax(25, MotorType.kBrushless);
    espeed = 0.50;
    intake1 = new SparkMax(21, MotorType.kBrushless);
    intake2 = new SparkMax(22, MotorType.kBrushless);
    coral = new SparkMax(20, MotorType.kBrushless);
    climb = new SparkMax(27,MotorType.kBrushless);
    stopClimb = new DigitalInput(0);

  }
  private static double algaeIntakeSpeed = 0.75;
  private static double algaeOutakeSpeed = 0.75;
  private static double algaeHoldspeed = 0;
  private static double elevatorHoldSpeed = 0.03;
  /**
   * Example command factory method.
   *
   * @return a command
   */
  

  public Command climbUp(){
   return runOnce(()->{climb.set(0.1);});
 }
  public Command climbDown(){
   return runOnce(()->{climb.set(-0.1);}
   );}
   public Command climbStop(){
   return runOnce(()->{climb.set(0);});}

  public Command coralright(){
    return run(()->{coral.set(0.5);});
  }
  public Command coralleft(){
    return run(()->{coral.set(-0.5);});}
  
  public Command coral(){
    return run(()->{help();});
  }
  public Command coralhelp(){
    return run(()->{helpme();});
  }
  public void help(){
    coral.set(-0.5);
  }
  public void helpme(){
    double neededval = coral.getEncoder().getPosition() - 10;
    if (coral.getEncoder().getPosition() < neededval){
      coral.set(-0.5);
    }
    else{coral.set(0);}
  }


  public Command coralstop(){return runOnce(()->{coral.set(0);});}
  //make the elevator go up
  public Command goUp() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          if (elevator1.getEncoder().getPosition() < 65.7 && elevator2.getEncoder().getPosition() > -66){
            elevator1.set(espeed);
            elevator2.set(-espeed);
          }
        });
  }
  //make the elevator go down
  public Command goDown(){
    return runOnce(() -> {
      if (elevator1.getEncoder().getPosition() > 0.07 && elevator2.getEncoder().getPosition() < -0.07)
      elevator1.set(-espeed);
      elevator2.set(espeed);
    });
  }

  //make the elevator stay in place
  public Command ElevatorHold(){
    return runOnce(() -> {
      elevator1.set(elevatorHoldSpeed);
      elevator2.set(-elevatorHoldSpeed);
    });
  }

  //algae intake
  public Command AlgaeIntake(){
    return runOnce(() -> {
      intake1.set(-algaeIntakeSpeed);
      intake2.set(algaeIntakeSpeed);
    });
  }

  //algae outtake
  public Command AlgaeOuttake(){
    return runOnce(() -> {
      intake1.set(algaeOutakeSpeed);
      intake2.set(-algaeOutakeSpeed);
    });
  }

  //algea hold
  public Command AlgaeHeld(){ // should be renamed algaeHold for readablility
    return runOnce(() -> {
      intake1.set(-algaeHoldspeed);
      intake2.set(algaeHoldspeed);
    });
  }

  public Command gotoLow(){
    return run(()->{gotoLevel(0.2, -0.2);});
  }
  public Command gotolevel1(){
    return run(()-> {gotoLevel(l1, l12);});
  }
  public Command gotolevel2(){
    return run(()-> {gotoLevel(l2, l22);});
  }
  public Command gotolevel3(){
    return run(()-> {gotoLevel(l3, l32);});
  }
  public Command gotolevel4(){
    return run(()-> {gotoLevel(l4, l42);});
  }
  public Command gotoAlgaelevel1(){
    return run(()-> {gotoLevel(a1, a12);});
  }
  public Command gotoAlgaelevel2(){
    return run(()-> {gotoLevel(a2, a22);});
  }

  //goto elevator level - get encoder values
  public void gotoLevel(double level, double level2){
    if (elevator1.getEncoder().getPosition() < level && elevator2.getEncoder().getPosition() > level2){
        elevator1.set(espeed);
        elevator2.set(-espeed);
    }
    else if (elevator1.getEncoder().getPosition() > level+1 && elevator2.getEncoder().getPosition() < level2-1){
      elevator1.set(-espeed);
        elevator2.set(espeed);
    }
    else{
      elevator1.set(elevatorHoldSpeed);
      elevator2.set(-elevatorHoldSpeed);
    }
  }
  
  //not un used currently, should slow down robot when proper uses added
  public BooleanSupplier slowItdown(){
    if (elevator1.getAbsoluteEncoder().getPosition() > 0.7){
      return ()->true;
    }
    else{
      return ()->false;
    }
  }

  @Override
  public void periodic() {
    //Throwing smartdashbaord values
    SmartDashboard.putNumber("test",elevator1.getEncoder().getPosition());
    SmartDashboard.putNumber("test2",elevator2.getEncoder().getPosition());
    SmartDashboard.putNumber("fuckoff", climb.getEncoder().getPosition());
    //SmartDashboard.putNumber("climb", climb.getPosition().getValueAsDouble());
    // This method will be called once per scheduler run
}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
