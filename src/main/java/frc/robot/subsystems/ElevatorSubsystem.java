// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new Elevator. */
  private final TalonFX motor = new TalonFX(ElevatorConstants.kMotorId,ElevatorConstants.kCanBus);
  private final TalonFX slave = new TalonFX(ElevatorConstants.kSlaveId,ElevatorConstants.kCanBus);
  private final DigitalInput limitswitch = new DigitalInput(ElevatorConstants.kDigitalPort);

  private States state = States.HOMING;
  private double position = 0;
  private double gain = 0;

  public enum States {
    HOME,
    HOMING,
    MOVING,
    ATPOSTION,
    ABORT
  }

  public ElevatorSubsystem() {
    motor.getConfigurator().apply(ElevatorConstants.kTalonFCFactoryDefaults);
    slave.getConfigurator().apply(ElevatorConstants.kTalonFCFactoryDefaults);
    motor.setNeutralMode(NeutralModeValue.Brake);
    slave.setNeutralMode(NeutralModeValue.Brake);
    slave.setControl(new Follower(ElevatorConstants.kMotorId, false));
  }

  public void setPosition (double cmdposition) {
    if (state == States.HOME) {
        position = cmdposition;
        state = States.MOVING;
    }
  }

  public void setHome() {
    if (limitswitch.get() == true) {
      motor.set(0.25);
      state = States.HOMING;
    }
    else state = States.HOME;
  }
  
  public void setAbort() {
    motor.set(0.0);
    state = States.ABORT;
  }

  public States getState(){
    return state; 
  } 

  private void control() {
    switch (state) {

      default:
      break;

      case HOME:
        state = States.HOME;
      break;

      case HOMING:
        if (limitswitch.get() == false) {
          motor.set(0.0);
          motor.setPosition(0.0);  
          position = 0.0; 
          state = States.HOME;
        }
        else state = States.HOMING;
      break;  

      case ABORT:
        state = States.ABORT;
      break;

      case MOVING: 
        if ((motor.getPosition().getValueAsDouble() <= position+gain) && (motor.getPosition().getValueAsDouble() >= position-gain)) {
            motor.set(-.01);            //The minimum amount of motor power needed to hold the elevator in position
            state = States.ATPOSTION;
        }
        if (motor.getPosition().getValueAsDouble() > position+gain) {   //We are too high so move downwards
            motor.set(1.0);
            state=States.MOVING; 
        }
        if (motor.getPosition().getValueAsDouble() < position-gain) {   //We are too low so move upwards
          motor.set(-1.0);
          state=States.MOVING;      
        }
      break;

      case ATPOSTION:
        state = States.ATPOSTION;
      break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    control();
    
    SmartDashboard.putNumber("Elevator Position",motor.getPosition().getValueAsDouble());
    
    switch (state) {
      default:
        SmartDashboard.putString("Elevator State","UNKNOWN");
      break;
      case HOME:
        SmartDashboard.putString("Elevator State","HOME");
      break;
      case HOMING:
        SmartDashboard.putString("Elevator State","HOMING");
      break;
      case MOVING:
        SmartDashboard.putString("Elevator State","MOVING");
      break;
      case ATPOSTION:
        SmartDashboard.putString("Elevator State","ATPOSITION");
      break;
      case ABORT:
        SmartDashboard.putString("Elevator State","ABORT");
      break;
    }
  }
}
