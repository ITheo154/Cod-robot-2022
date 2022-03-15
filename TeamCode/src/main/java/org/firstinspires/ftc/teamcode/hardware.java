package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class hardware {

    private HardwareMap hardwareMap;

    hardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }
    //movement motors
    DcMotor getMotorstangafata(){return hardwareMap.dcMotor.get("stangafata");}
    DcMotor getMotordreaptafata(){return hardwareMap.dcMotor.get("dreaptafata");}
    DcMotor getMotorstangaspate(){return hardwareMap.dcMotor.get("stangaspate");}
    DcMotor getMotordreaptaspate(){return hardwareMap.dcMotor.get("dreaptaspate");}

    //dc motors
    DcMotor getIntake1(){return hardwareMap.dcMotor.get("intake1");}
    DcMotor getIntake2(){return hardwareMap.dcMotor.get("intake2");}
    DcMotor getExtindere(){return hardwareMap.dcMotor.get("extindere");}

    //servos
    DcMotor getCarusel(){return hardwareMap.dcMotor.get("carusel");}
    Servo getRetragere1(){return hardwareMap.servo.get("retragere1");}
    Servo getRetragere2(){return hardwareMap.servo.get("retragere2");}
    Servo getControl(){return hardwareMap.servo.get("control");}
    Servo getCutie(){return hardwareMap.servo.get("cutie");}

    DistanceSensor getDistanta(){return hardwareMap.get(DistanceSensor.class, "distanta");}

    BNO055IMU getGyro() {return hardwareMap.get(BNO055IMU.class, "imu");}
}
