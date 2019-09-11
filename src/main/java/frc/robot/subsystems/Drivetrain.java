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



  private CANPIDController mPIDLeft  = new CANPIDController(mMasterLeft);
  private CANPIDController mPIDRight = new CANPIDController(mMasterRight);
  public double kP,kD,kI;
  private static Drivetrain instance;

  private Drivetrain()
  {
    mLeftSlaveA.follow(mMasterLeft);
    mRightSlaveA.follow(mMasterRight);

    mMasterRight.setInverted(true);
    mRightSlaveA.setInverted(true);
    kP = 10;
    kI = 0;
    kD = 0;
    mPIDLeft.setP(kP);
    mPIDLeft.setI(kI);
    mPIDLeft.setD(kD);
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
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void testAutonomous () {
    mPIDLeft.setReference(10, ControlType.kPosition);
    mPIDRight.setReference(10, ControlType.kPosition);
  }

  
}
