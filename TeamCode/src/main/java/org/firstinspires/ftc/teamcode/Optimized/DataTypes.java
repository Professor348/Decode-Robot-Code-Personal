package org.firstinspires.ftc.teamcode.Optimized;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;

public class DataTypes {
    public static class PIDFFCoefficients {
        public double kP = 0;
        public double kI = 0;
        public double kD = 0;
        public double kV = 0;
        public double kA = 0;
        public double kS = 0;

        public PIDFFCoefficients(){}
        public PIDFFCoefficients(double kP){
            this.kP = kP;
        }
        public PIDFFCoefficients(double kP, double kI){
            this(kP);
            this.kI = kI;
        }
        public PIDFFCoefficients(double kP, double kI, double kD){
            this(kP, kI);
            this.kD = kD;
        }
        public PIDFFCoefficients(double kP, double kI, double kD, double kV){
            this(kP, kI, kD);
            this.kV = kV;
        }
        public PIDFFCoefficients(double kP, double kI, double kD, double kV, double kA){
            this(kP, kI, kD, kV);
            this.kA = kA;
        }
        public PIDFFCoefficients(double kP, double kI, double kD, double kV, double kA, double kS){
            this(kP, kI, kD, kV, kA);
            this.kS = kS;
        }

        public PIDCoefficients getPIDCoefficients(){
            return new PIDCoefficients(kP, kI, kD);
        }
        public BasicFeedforwardParameters getFFCoefficients(){
            return new BasicFeedforwardParameters(kV, kA, kS);
        }
    }
    public static enum AngleUnit {
        RADIANS,
        DEGREES,
        TICKS
    }
}
