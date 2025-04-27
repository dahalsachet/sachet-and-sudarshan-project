package linefollower;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower {

    public static void main(String[] args) {
        
        // Initialize the color sensor on port S4
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
        SampleProvider redMode = colorSensor.getRedMode();  // Reflected light intensity mode

        // Array to hold sensor sample
        float[] sample = new float[redMode.sampleSize()];

        // Set motor base speed
        int baseSpeed = 300;
        Motor.A.setSpeed(baseSpeed);
        Motor.B.setSpeed(baseSpeed);

        // Start motors
        Motor.A.forward();
        Motor.B.forward();

        // Threshold to detect black (you may need to adjust it based on your surface!)
        float blackThreshold = 0.2f;

        while (!Button.ESCAPE.isDown()) {
            // Fetch light intensity sample
            redMode.fetchSample(sample, 0);
            float lightIntensity = sample[0];

            // Display intensity for debugging
            LCD.clear();
            LCD.drawString("Light: " + (int)(lightIntensity * 100) + "%", 0, 0);

            // Line following logic
            if (lightIntensity < blackThreshold) {
                // On black line → go straight
                Motor.A.setSpeed(baseSpeed);
                Motor.B.setSpeed(baseSpeed);
            } else {
                // Off black line → correct course
                // You can decide the turning sharpness
                if (lightIntensity > 0.6) {
                    // Very bright (white surface) → sharp left
                    Motor.A.setSpeed(100);  // Left slower
                    Motor.B.setSpeed(400);  // Right faster
                } else {
                    // Somewhere between → gentle right
                    Motor.A.setSpeed(400);  // Left faster
                    Motor.B.setSpeed(100);  // Right slower
                }
            }

            // Move motors forward
            Motor.A.forward();
            Motor.B.forward();

            // Short delay to stabilize
            Delay.msDelay(50);
        }

        // Stop motors and close sensor
        Motor.A.stop();
        Motor.B.stop();
        colorSensor.close();
    }
}
