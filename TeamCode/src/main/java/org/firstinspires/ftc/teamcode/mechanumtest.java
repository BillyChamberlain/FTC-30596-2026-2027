package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class mechanumtest extends OpMode {
    private DcMotor RF;
    private DcMotor LF;
    private DcMotor RB;
    private DcMotor LB;

    private DcMotor intake;

    private DcMotor trans;

    private DcMotor turret_shoot;
    private DcMotor turret_aim;

    private final float flyconstant = 1f; // TODO: NEEDS TO BE CHANGED

    private final float flywheel_dia = 0.1202436f;

    //TODO: Change these vals to match field size
    private final double slowspeedlen = 1;
    private final double slowspeedspeed = 3000 / 60.0;


    private final double medspeedlen = 2;
    private final double medspeedspeed = 4000 / 60.0;

    private final double fastspeedlen = 3;
    private final double fastspeedspeed = 5000 / 60.0;

    public void init(){
        //Drive
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");

        //TODO: Change
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        //Other mechs that shouldn't need to be changed
        intake = hardwareMap.get(DcMotor.class, "intake");
        trans = hardwareMap.get(DcMotor.class, "transfer");
        turret_shoot = hardwareMap.get(DcMotor.class, "shoot");
        turret_aim = hardwareMap.get(DcMotor.class, "aim");

        LimelightCam ll = new LimelightCam(hardwareMap);
    }
    public void loop(){
        double vertical = -gamepad1.left_stick_y;
        double horizontal = gamepad1.left_stick_x;
        double pivot = gamepad1.right_stick_x;

        //Change + and - on this if necessary
        RF.setPower(pivot + (-vertical + horizontal));
        RB.setPower(pivot + (-vertical - horizontal));
        LF.setPower(-pivot + (-vertical - horizontal));
        LB.setPower(-pivot + (-vertical + horizontal));
    }

    private static double calculate_fly_speed(float flylen, double x, double y, double ang, Telemetry telemetry){
        try {
            double numerator = 9.8 * Math.pow(x, 2.0);
            double denominator = 2.0 * Math.pow(Math.cos(Math.toRadians(ang)), 2) * (x * Math.tan(Math.toRadians(ang)) - y);
            return ((1 / flylen) * Math.pow((numerator / denominator), 0.5));
        }
        catch (Exception e){
            telemetry.addLine("Error: Cannot shoot from here, you are likely too far away or smthn idk");
            telemetry.update();
            return -1;
        }
    }

    private static double[] angleFromRPS(
            double flywheelRPS,
            double flywheelDiameterMeters,
            double gripConstant,
            double horizontalDistanceMeters, //x
            double heightDifferenceMeters //y
    ) {
        final double g = 9.81;

        double ballVelocity =
                gripConstant * Math.PI * flywheelDiameterMeters * flywheelRPS;

        double vSquared = ballVelocity * ballVelocity;

        // Quadratic in tan(theta):
        // A*tan(theta)^2 + B*tan(theta) + C = 0
        double A = (g * horizontalDistanceMeters * horizontalDistanceMeters)
                / (2.0 * vSquared);

        double B = -horizontalDistanceMeters;
        double C = A + heightDifferenceMeters;

        double discriminant = B * B - 4.0 * A * C;

        if (discriminant < 0) {
            return new double[0]; // RPS is too low for this target.
        }

        double lowTan = (-B - Math.sqrt(discriminant)) / (2.0 * A);
        double highTan = (-B + Math.sqrt(discriminant)) / (2.0 * A);

        double lowAngleDegrees = Math.toDegrees(Math.atan(lowTan));
        double highAngleDegrees = Math.toDegrees(Math.atan(highTan));

        return new double[] {lowAngleDegrees, highAngleDegrees};
    }
}
