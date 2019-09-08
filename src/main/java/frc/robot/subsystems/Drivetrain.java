/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Utils.Constants;

/**
 * Add your docs here.
 */
public class Drivetrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private CANSparkMax mMasterLeft  = new CANSparkMax(RobotMap.leftMaster, MotorType.kBrushless);
  private CANSparkMax mLeftSlaveA  = new CANSparkMax(RobotMap.leftSlaveA, MotorType.kBrushless);
  private CANSparkMax mMasterRight = new CANSparkMax(RobotMap.rightMaster,MotorType.kBrushless);
  private CANSparkMax mRightSlaveA = new CANSparkMax(RobotMap.rightSlaveA,MotorType.kBrushless);
  private double m_quickStopAccumulator = 0;
  private double m_quickStopAlpha = 0.1;



  private CANPIDController mPIDLeft  = new CANPIDController(mMasterLeft);
  private CANPIDController mPIDRight = new CANPIDController(mMasterRight);

  private static Drivetrain instance;

  private Drivetrain()
  {
    mLeftSlaveA.follow(mMasterLeft);
    mRightSlaveA.follow(mMasterRight);

    mMasterRight.setInverted(true);
    mRightSlaveA.setInverted(true);
  }

 /**
   * @return the instance
   */
  public static Drivetrain getInstance() {
    if(instance == null)
    {
      instance = new Drivetrain();
    }
    return instance;
  }


  public void cartesianDrive(double x, double y)
  {
    double tempX = (Constants.kDriveSensFactor*Math.pow(x,3)) + ((1 - Constants.kDriveSensFactor)*x);
    double tempY = (Constants.kDriveSensFactor*Math.pow(y,3)) + ((1 - Constants.kDriveSensFactor)*y);

    double powerLeft  = tempY - tempX;
    double powerRight = tempY + tempX;

    mPIDLeft.setReference(powerLeft, ControlType.kDutyCycle);
    mPIDRight.setReference(powerRight, ControlType.kDutyCycle);
  }

  public void curvatureDrive(double xSpeed, double y, boolean isQuickTurn)
  {
    boolean overPower = false;
    double throttle;
    double angularPow;
    xSpeed = Constants.applyDeadband(xSpeed, Constants.kDriveDeadBand);
    y = Constants.applyDeadband(y, Constants.kDriveDeadBand);

    if(isQuickTurn)
    {
       if(Math.abs(xSpeed) < 0.2)
       {
        m_quickStopAccumulator = (1 - m_quickStopAlpha)*m_quickStopAccumulator + m_quickStopAlpha * Constants.clamp(y, -1.0, 1.0)*2;
       }
       overPower = true;
       angularPow = y;
    }else
    {
      overPower = false;
      angularPow = Math.abs(xSpeed) * y - m_quickStopAccumulator;

      if(m_quickStopAccumulator > 1)
      {
        m_quickStopAccumulator -= 1;
      }else if(m_quickStopAccumulator < -1)
      {
        m_quickStopAccumulator += 1;
      }else
      {
        m_quickStopAccumulator = 0.0;
      }

      
    }

    double leftPow = xSpeed + angularPow;
    double rightPow = xSpeed - angularPow;

    if (overPower) {
      if (leftPow > 1.0) {
        rightPow -= leftPow - 1.0;
        leftPow = 1.0;
      } else if (rightPow > 1.0) {
        leftPow -= rightPow - 1.0;
        rightPow = 1.0;
      } else if (leftPow < -1.0) {
        rightPow -= leftPow + 1.0;
        leftPow = -1.0;
      } else if (rightPow < -1.0) {
        leftPow -= rightPow + 1.0;
        rightPow = -1.0;
      }

    mPIDLeft.setReference(leftPow, ControlType.kDutyCycle);
    mPIDRight.setReference(rightPow, ControlType.kDutyCycle);
  }

  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
