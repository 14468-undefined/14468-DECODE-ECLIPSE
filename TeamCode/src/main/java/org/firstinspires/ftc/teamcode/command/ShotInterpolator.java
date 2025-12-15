package org.firstinspires.ftc.teamcode.command;



public class ShotInterpolator {

    private final ShotPoint[] LUT = {
            new ShotPoint(18.0, 32, 3400),
            new ShotPoint(15.0, 36, 3800),
            new ShotPoint(12.0, 41, 4200),
            new ShotPoint(9.0,  47, 4600)
    };

    public ShotPoint interpolate(double camAngle) {

        // Clamp
        if (camAngle >= LUT[0].camAngle) return LUT[0];
        if (camAngle <= LUT[LUT.length - 1].camAngle)
            return LUT[LUT.length - 1];


        for (int i = 0; i < LUT.length - 1; i++) {
            ShotPoint a = LUT[i];
            ShotPoint b = LUT[i + 1];

            if (camAngle <= a.camAngle && camAngle >= b.camAngle) {
                double t = (camAngle - a.camAngle) / (b.camAngle - a.camAngle);

                double hood = a.hoodDeg + t * (b.hoodDeg - a.hoodDeg);
                double rpm  = a.rpm     + t * (b.rpm     - a.rpm);

                return new ShotPoint(camAngle, hood, rpm);
            }
        }

        return LUT[0]; // fallback
    }
}
