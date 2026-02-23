package org.firstinspires.ftc.teamcode.Optimized;

public class Utilities {
    public static double degreesToTicks(double degrees, double encoderResolution){
        return (degrees * encoderResolution/360);
    }
    public static double radiansToTicks(double radians, double encoderResolution){
        return (radians * encoderResolution/(2*Math.PI));
    }
    public static double ticksToDegrees(double ticks, double encoderResolution){
        return (ticks/encoderResolution) * 360;
    }
    public static double ticksToRadians(double ticks, double encoderResolution){
        return (ticks/encoderResolution) * (2*Math.PI);
    }
}
