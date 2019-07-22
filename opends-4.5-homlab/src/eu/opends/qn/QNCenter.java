/*

*/

package eu.opends.qn;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;


/**
 * sending and receiving messages between OpenDS and QN-ACTR
 *
 * @author 
 */
public class QNCenter
{

//	private static Simulator sim;
//	private static boolean sendDataToHMI = false;

	private DatagramSocket senderSocket;
	private DatagramSocket receiveSocket;
	private int destPort = 8765; 
	private int recvPort = 5678;
	
	private final int MSG_PARTS = 5;
	
	private final int QNCLOCK = 0;
	private final int ACCELERATOR = 1;
	private final int BRAKE = 2;
	private final int STEERING = 3;
	private final int REPORT = 4;
	private final int bufferSizefromQN = 1024;
	private final int bufferSizetoQN = 8192;
	
	private String msg_prefix[] = {
			"QNClock: ",
			"Accelerator: ",
			"Brake: ",
			"Steering: ",
			"report: "
	};
	
	// for performance improvement
	private int msg_prefix_len[] = {0, 0, 0, 0, 0};


	public QNCenter() {
		try {
			receiveSocket = new DatagramSocket(recvPort);
			receiveSocket.setBroadcast(true);
			receiveSocket.setReuseAddress(true);
		} catch (SocketException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		try {
			senderSocket = new DatagramSocket(10);
			senderSocket.setBroadcast(true);
			senderSocket.setReuseAddress(true);
		} catch (SocketException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		for(int i = 0; i<MSG_PARTS; i++) {
			this.msg_prefix_len[i] = this.msg_prefix[i].length();
		}
	}
	
	public void send(String msg) {
		byte buffer[] = new byte[bufferSizetoQN]; 
		buffer = msg.getBytes(); 
		//System.out.println("sending msg: " + msg);
		//DatagramPacket packet = new DatagramPacket(buffer, buffer.length, InetAddress.getLocalHost(), destPort);
		DatagramPacket packet = null;
		try {
			if(buffer.length > bufferSizetoQN) System.err.println("error: send msg longer than buffer size: " + bufferSizetoQN);
			packet = new DatagramPacket(buffer, buffer.length, InetAddress.getByName("localhost"), destPort);
		} catch (UnknownHostException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		try {
			if(packet != null) this.senderSocket.send(packet);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		//System.out.println(System.currentTimeMillis() + ", msg sended: " + msg);
	}
	
	public QNControlRecv receive() {
		//System.out.println(System.currentTimeMillis() + ", server listening");

		byte buffer[] = new byte[bufferSizefromQN]; 
		DatagramPacket packet = new DatagramPacket(buffer, buffer.length); 
		try {
			receiveSocket.receive(packet);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} 
		//System.out.println(System.currentTimeMillis() + ", packet received: " +  new String(packet.getData()));
		return msgToQNControlRecv(new String(packet.getData()));
	}
	
	private QNControlRecv msgToQNControlRecv(String msg) {
		
		QNControlRecv ret = new QNControlRecv();
		
		String[] tokens = msg.split(", "); 
		
		if(tokens.length < this.MSG_PARTS) {
			System.err.println("wrong msg format received from QN-ACTR");
		}
		else {
			ret.setQNClock(Float.parseFloat(tokens[this.QNCLOCK].substring(this.msg_prefix_len[this.QNCLOCK])));
			ret.setAccelerator(Float.parseFloat(tokens[this.ACCELERATOR].substring(this.msg_prefix_len[this.ACCELERATOR])));
			ret.setBrake(Float.parseFloat(tokens[this.BRAKE].substring(this.msg_prefix_len[this.BRAKE])));
			ret.setSteering(Float.parseFloat(tokens[this.STEERING].substring(this.msg_prefix_len[this.STEERING])));
			// notice that the msg ends with ',' so there's no need to see the last token as a special case
			ret.setReport(Boolean.parseBoolean(tokens[this.REPORT].substring(this.msg_prefix_len[this.REPORT])));
			
		}
	    
	    //System.out.println("QNControlRecv instance received: " + ret);    
	    return ret;
	}


/*	private static void receiveMessage() {

		byte buffer[] = new byte[500];
		DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
		try {
			receiveSocket.receive(packet);
		} catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		String receivedString = new String(packet.getData());
		System.out.println("DATA RECEIVED: " + receivedString);

		Matcher m1 = Pattern.compile("QNClock: [-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?").matcher(receivedString);
		Matcher m2 = Pattern.compile("Accelerator: [-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?").matcher(receivedString);
		Matcher m3 = Pattern.compile("Brake: [-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?").matcher(receivedString);
		Matcher m4 = Pattern.compile("Steering: [-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?").matcher(receivedString);

	    double QNClock;
		if( m1.find() ) QNClock = Double.parseDouble(m1.group().substring("QNClock: ".length()));
	    double Accelerator = 0.0f;
		if( m2.find() ) Accelerator = Double.parseDouble(m2.group().substring("Accelerator: ".length()));
	    double Brake = 0.0f;
		if( m3.find() ) Brake = Double.parseDouble(m3.group().substring("Brake: ".length()));
	    double steering = 0.0f;
		if( m4.find() ) steering = Double.parseDouble(m4.group().substring("Steering: ".length()));

		System.out.println("DATA RECEIVED: " + receivedString);
		SteeringCar car = sim.getCar();
		car.setAcceleratorPedalIntensity((-1 * (float)Accelerator));
		car.setBrakePedalIntensity((float)Brake);
		
//		steering = steering * FastMath.DEG_TO_RAD;
//		car.setSteeringWheelState((float)steering);
		
		car.steerWithQN((float)steering);
//		car.getTransmission().performAcceleration((float)Accelerator);
//		car.getCarControl().steer(0-(float)steering);
		//car.getCarControl().accelerate((0 - (float)Accelerator)/2);
		//car.getCarControl().accelerate((float)Brake);
//		car.steer((float)steering);
//		car.setMaxSpeed(150);

		//sim.getSceneNode().getChild(0).getWorldTransform().getTranslation()

//		sim.getCar().getCarControl().set

	}
*/

}