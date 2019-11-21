
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
/*
public void read(int regAddress, int numBytes, byte readData[]) {

	// local byte array to cast register address into.
	byte[] reg2read = new byte[1];

	// Avoid inherent repeat-start sequence used in WPILib I2C base class
	// "read" API; repeat-start (normally used to avoid bus arbitration
	// loss on a multi-master I2C bus) is not supported by this device.
	// Instead, write the read address separately, followed by explicit raw
	// byte reads.  Currently, only single-byte or two-byte reads are supported.

	// check byte count
	if (numBytes > 2) {
		System.out.println("bad lidar read byte count");
	}

	// OR auto-indexing bit into address if needed
	if (numBytes > 1)
		regAddress |= LL3_AUTOINCREMENT_ADDRESSING;

	// downcast read address to byte array required by writeBulk() API
	reg2read[0] = (byte)regAddress;

	// bulk write read address separately (avoid unsupported repeat-start read sequence)
	if (m_i2c.writeBulk(reg2read, 1)) {
		System.out.println("bad lidar m_i2c.writeBulk() result");
	}

	// raw data read (without read address)
	if (m_i2c.readOnly(readData, numBytes)) {
		System.out.println("bad lidar m_i2c.readOnly() result");
	}
}
*/
}