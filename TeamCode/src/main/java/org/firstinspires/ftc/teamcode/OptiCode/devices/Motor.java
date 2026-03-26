package org.firstinspires.ftc.teamcode.OptiCode.devices;

import static org.firstinspires.ftc.teamcode.OptiCode.Data.currentRunningOpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.skeletonarmy.marrow.OpModeManager;

public class Motor {

    public static enum RPM{
        GOBILDA_6000,
        GOBILDA_1620,
        GOBILDA_1150,
        GOBILDA_435,
        GOBILDA_312,
        GOBILDA_223,
        GOBILDA_117,
        GOBILDA_84,
        GOBILDA_60,
        GOBILDA_43,
        GOBILDA_30
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
        switch (motorRPM){
            case GOBILDA_6000:
                this.ticksPerRotation = 28;
                break;
            case GOBILDA_1620:
                this.ticksPerRotation = 103.8;
                break;
            case GOBILDA_1150:
                this.ticksPerRotation = 145.1;
                break;
            case GOBILDA_435:
                this.ticksPerRotation = 384.5;
                break;
            case GOBILDA_312:
                this.ticksPerRotation = 537.7;
                break;
            case GOBILDA_223:
                this.ticksPerRotation = 751.8;
                break;
            case GOBILDA_117:
                this.ticksPerRotation = 1425.1;
                break;
            case GOBILDA_84:
                this.ticksPerRotation = 1993.6;
                break;
            case GOBILDA_60:
                this.ticksPerRotation = 2786.2;
                break;
            case GOBILDA_43:
                this.ticksPerRotation = 3895.9;
                break;
            case GOBILDA_30:
                this.ticksPerRotation = 5281.1;
                break;

        }
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
