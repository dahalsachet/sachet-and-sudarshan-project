package lightsensor;

import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;

public class LightSensor
{
    public static void main(String[] args)
    {
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
        
        // Use reflected light (not ambient light)
        SampleProvider light = colorSensor.getRedMode();
        
        float[] sample = new float[light.sampleSize()];
        
        LCD.clear();
        LCD.drawString("Reading Light...", 0, 0);

        while (!Button.ESCAPE.isDown()) // Exit if ESCAPE button is pressed
        {
            // Fetch the reflected light intensity
            light.fetchSample(sample, 0);
            
            // Display the light intensity value on the LCD screen
            LCD.clear();
            int lightIntensity = (int)(sample[0] * 100); // Convert to percentage
            LCD.drawString("Light Intensity:", 0, 0);
            LCD.drawString(lightIntensity + "%", 0, 1);
            
            try 
            {
                Thread.sleep(100);
            } 
            catch (InterruptedException e) 
            {
                e.printStackTrace();
            }
        }
        
        colorSensor.close();
    }
}
