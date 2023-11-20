package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DigitalSensor extends SubsystemBase {
    //private final AnalogInput m_sensor;
    //private final AnalogInput m_sensor2;
    private final DigitalInput sens0; // Port 0
    private final DigitalInput sens1; // Port 1
    public boolean override;

    public DigitalSensor() {
        override = false;
        //m_sensor = new AnalogInput(0);
        //m_sensor2 = new AnalogInput(1);
        sens0 = new DigitalInput(0);
        sens1 = new DigitalInput(1);
    }

    /*

    public boolean sensorBoolean0(){
        if(override) return false;
        return getSensor0AvValue();
        //return m_sensor.getAverageValue() <= 7;
    }

    public boolean sensorBoolean1(){
        if(override) return false;
        //return m_sensor2.getAverageValue() <= 7;
        return getSensor1AvValue();
    }
    */



    /**
     *
     * @return Double - Sensor 0's average value
     */
    public boolean getSens0() {
        return !sens0.get();
    }

    /**
     *
     * @return Double - Sensor 1's average value
     */
    public boolean getSens1() {
        return !sens1.get();
    }

    /**
     *
     * @return Boolean / If the color sensors have something in front of them - True = There is something in front of the color sensor.
     */



    @Override
    public void periodic() {

    }
}