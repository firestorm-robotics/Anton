package frc.robot.Utils;

public class Constants
{
    public static double kDriveSensFactor = 0.6;
    public static final double kDriveDeadBand = 0.02;
    public static double applyDeadband(double x, double deadBand)
    {
        if(Math.abs(x) < deadBand)
        {
            return 0;
        }
        return x;
    }

    public static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(value, high));
      }
}