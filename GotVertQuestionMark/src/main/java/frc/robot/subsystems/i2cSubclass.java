
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.TimedRobot;

public class i2cSubclass extends TimedRobot {
    private static I2C Wire = new I2C(Port.kOnboard, 4);
    private static final int MAX_BYTES = 32;
	


//methods

public void portBegin(String input) {
    char[] CharArray = input.toCharArray();
    byte[] WriteData = new byte[CharArray.length];

    
}

private String read(){//function to read the data from arduino
    byte[] data = new byte[MAX_BYTES];//create a byte array to hold the incoming data
    Wire.read(4, MAX_BYTES, data);//use address 4 on i2c and store it in data
    String output = new String(data);//create a string from the byte array
    int pt = output.indexOf((char)255);
    return (String) output.subSequence(0, pt < 0 ? 0 : pt);//im not sure what these last two lines do
                                                           //sorry :(
}


}