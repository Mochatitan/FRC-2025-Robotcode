// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.commands.coralPlace;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.commands.coralPlace;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class SimpleElevatorSubsystem extends SubsystemBase {

  public SparkMax elevatorMotor1 = new SparkMax(Constants.NonChassis.elevatorMotorID1, MotorType.kBrushless);
  public SparkMax elevatorMotor2 = new SparkMax(Constants.NonChassis.elevatorMotorID2, MotorType.kBrushless);
  private RelativeEncoder encoder1 = elevatorMotor1.getEncoder();
  private RelativeEncoder encoder2 = elevatorMotor2.getEncoder();

  private PIDController m_pid = new PIDController(0.04, 0,0.01);

  DigitalInput input = new DigitalInput(1);

  public SimpleElevatorSubsystem() {
    m_pid.setTolerance(0.3);
    //Make sure the motors are on brake mode in your tuner software
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetEnc() {
    encoder1.setPosition(0.2);
    encoder2.setPosition(0.2);
  }

  public void elevatorStop()
  {
    setSpeed(0);
  }
  private void setSpeed(double speed){
    elevatorMotor1.set(speed);
    elevatorMotor2.set(-speed); //This way you can set them both at once 
  } 

  public boolean checkPhotoeye()
  {
    SmartDashboard.putNumber("encoder position Motor 1",getPosition());
    SmartDashboard.putNumber("encoder position Motor 2",getPositionEncoder2());
    SmartDashboard.putNumber("encoder error", getError());
    SmartDashboard.putBoolean("Photoeye",input.get());
    return input.get();
  }

  public double getPosition() {
    return encoder1.getPosition();
  }

  public double getPositionEncoder2() {
    return encoder2.getPosition();
  }

  /**
   * @return The difference between the 2 elevator encoders
   */
  public double getError(){
    return Math.abs(getPosition()-getPositionEncoder2());
  }

    public void moveToL2(){
        setSpeed(m_pid.calculate(Constants.NonChassis.ticksToL2, getPosition()));
    }
    public void moveToL3(){
        setSpeed(m_pid.calculate(Constants.NonChassis.ticksToL3, getPosition()));
    }
    public void moveToL4(){
        setSpeed(m_pid.calculate(Constants.NonChassis.ticksToL4, getPosition()));
    }
    
    public Command moveToL2Command(){
        return runOnce(() -> moveToL2());
    }
    public Command moveToL3Command(){
        return runOnce(() -> moveToL3());
    }
    public Command moveToL4Command(){
        return runOnce(() -> moveToL4());
    }

}
