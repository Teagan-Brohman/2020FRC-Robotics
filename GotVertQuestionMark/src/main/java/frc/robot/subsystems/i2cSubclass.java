
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.TimedRobot;
import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class i2cSubclass extends IterativeRobot {
	private static Rev2mDistanceSensor distOnboard;

	// private static I2C Wire = new I2C(Port.kOnboard, 4);
	// private static final int MAX_BYTES = 32;

	// while(true){
	// Wire.read(, count, buffer)
	// }

	// methods
	@Override
	public void robotInit() {
		/**
		 * Rev 2m distance sensor can be initialized with the Onboard I2C port or the
		 * MXP port. Both can run simultaneously.
		 */
		distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
	}

	public static void getRange() {

		
		Rev2mDistanceSensor distOnboard = new Rev2mDistanceSensor(Port.kOnboard);

		if (distOnboard.isRangeValid()) {
		SmartDashboard.putNumber("Range Onboard", distOnboard.getRange());
		SmartDashboard.putNumber("Timestamp Onboard", distOnboard.getTimestamp());
	  }
  
}

public void disabledInit() {
    /**
     * The background thread may be stopped by calling setAutomaticMode(false).
     * This will command any active sensors to terminate current measurements
     * and the thread will stop.
     */
    distOnboard.setAutomaticMode(false);
  }

// public void MuxPortBegin(byte deviceAddress) 
//  {
	
//  	if(Wire.write(0x70, deviceAddress)){
		
// 	 }
// 	 else{

// 	 } //Write to Board Specifying Port
	
//  }



}

// private static byte[] fromHexString(final String encoded) {
//     if ((encoded.length() % 2) != 0)
//         throw new IllegalArgumentException("Input string must contain an even number of characters");

//     final byte result[] = new byte[encoded.length()/2];
//     final char enc[] = encoded.toCharArray();
//     for (int i = 0; i < enc.length; i += 2) {
//         StringBuilder curr = new StringBuilder(2);
//         curr.append(enc[i]).append(enc[i + 1]);
//         result[i/2] = (byte) Integer.parseInt(curr.toString(), 16);
//     }
//     return result;
// }


 
// }



// Code Converted from C to Java

// private String read(){//function to read the data from arduino
//     byte[] data = new byte[MAX_BYTES];//create a byte array to hold the incoming data
//     Wire.read(4, MAX_BYTES, data);//use address 4 on i2c and store it in data
//     String output = new String(data);//create a string from the byte array
//     int pt = output.indexOf((char)255);
//     return (String) output.subSequence(0, pt < 0 ? 0 : pt);//im not sure what these last two lines do
//                                                            //sorry :(
// }
// /*
// public void read(int regAddress, int numBytes, byte readData[]) {

// 	// local byte array to cast register address into.
// 	byte[] reg2read = new byte[1];

// 	// Avoid inherent repeat-start sequence used in WPILib I2C base class
// 	// "read" API; repeat-start (normally used to avoid bus arbitration
// 	// loss on a multi-master I2C bus) is not supported by this device.
// 	// Instead, write the read address separately, followed by explicit raw
// 	// byte reads.  Currently, only single-byte or two-byte reads are supported.

// 	// check byte count
// 	if (numBytes > 2) {
// 		System.out.println("bad lidar read byte count");
// 	}

// 	// OR auto-indexing bit into address if needed
// 	if (numBytes > 1)
// 		regAddress |= LL3_AUTOINCREMENT_ADDRESSING;

// 	// downcast read address to byte array required by writeBulk() API
// 	reg2read[0] = (byte)regAddress;

// 	// bulk write read address separately (avoid unsupported repeat-start read sequence)
// 	if (m_i2c.writeBulk(reg2read, 1)) {
// 		System.out.println("bad lidar m_i2c.writeBulk() result");
// 	}

// 	// raw data read (without read address)
// 	if (m_i2c.readOnly(readData, numBytes)) {
// 		System.out.println("bad lidar m_i2c.readOnly() result");
// 	}
// }
// */
// }