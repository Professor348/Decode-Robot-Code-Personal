package org.firstinspires.ftc.teamcode.OptiCode.devices;

import static org.firstinspires.ftc.teamcode.OptiCode.Data.currentRunningOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.skeletonarmy.marrow.OpModeManager;

public class Motor {

    public enum RPM{
        GOBILDA_6000 {
            @Override
            double getTPR() {
                return 28;
            }
        },
        GOBILDA_1620 {
            @Override
            double getTPR() {
                return 103.8;
            }
        },
        GOBILDA_1150 {
            @Override
            double getTPR() {
                return 145.1;
            }
        },
        GOBILDA_435 {
            @Override
            double getTPR() {
                return 384.5;
            }
        },
        GOBILDA_312 {
            @Override
            double getTPR() {
                return 537.7;
            }
        },
        GOBILDA_223 {
            @Override
            double getTPR() {
                return 751.8;
            }
        },
        GOBILDA_117 {
            @Override
            double getTPR() {
                return 1425.1;
            }
        },
        GOBILDA_84 {
            @Override
            double getTPR() {
                return 1993.6;
            }
        },
        GOBILDA_60 {
            @Override
            double getTPR() {
                return 2786.2;
            }
        },
        GOBILDA_43 {
            @Override
            double getTPR() {
                return 3895.9;
            }
        },
        GOBILDA_30 {
            @Override
            double getTPR() {
                return 5281.1;
            }
        };

        abstract double getTPR();
    }
    public enum AngleUnit {
        TICKS,
        DEGREES,
        RADIANS,
        ROTATIONS
    }

    private OpMode opmode;
    private DcMotorEx motor;
    private final String deviceName;
    private double ticksPerRotation = 28;

    Motor(String deviceName){
        this.deviceName = deviceName;
    }
    Motor(String deviceName, double ticksPerRotation){
        this.deviceName = deviceName;
        this.ticksPerRotation = ticksPerRotation;
    }
    Motor(String deviceName, RPM motorRPM){
        this.deviceName = deviceName;
        this.ticksPerRotation = motorRPM.getTPR();
    }

    /**
     * Initializes the hardware of the motor.
     */
    public void init(){
        opmode = OpModeManager.getActiveOpMode();
        motor = currentRunningOpMode.hardwareMap.get(DcMotorEx.class, deviceName);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void reZero(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Applies a amount of power to the motor*/
    public void setPower(double power){motor.setPower(power);}
    /** @return The current power applied to the motor*/
    public double getPower() {return motor.getPower();}
}
