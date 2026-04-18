using System;
using Godot;

public class Client : Node
{
	private PacketPeerUDP _udp;
	private int _port = 8881;
	TankStatus localUdpSubscriber;

	float decodedEulerX;
	float decodedEulerY;
	float decodedEulerZ;

	public override void _Ready()
	{
		TankStatusNative.constructTankStatus(ref localUdpSubscriber);
		_udp = new PacketPeerUDP();

		// Listen directly on the port for incoming UDP datagrams
		Error err = _udp.Listen(_port);

		if (err == Error.Ok)
		{
			GD.Print($"UDP Server listening on port {_port}...");
		}
		else
		{
			GD.PrintErr($"Failed to start UDP listener. Error code: {err}");
		}
	}

	public override void _Process(float delta)
	{
		// Continuously check if any packets have arrived
		while (_udp.GetAvailablePacketCount() > 0)
		{
			// Grab the raw bytes
			byte[] packet = _udp.GetPacket();

			// Initialize a fresh struct
			TankStatus ts = new TankStatus();

			// Optional: Call your C constructor if it sets default values
			TankStatusNative.constructTankStatus(ref ts);

			// Pass the byte array and the struct reference to the C library
			// The C library will read the bytes and populate the fields in 'ts'
			TankStatusNative.readByteTankStatus(packet, packet.Length, ref ts);
			decodedEulerX = TankStatusNative.getDecodedShortToFloat(ts.eulerX);
			decodedEulerY = TankStatusNative.getDecodedShortToFloat(ts.eulerY);
			decodedEulerZ = TankStatusNative.getDecodedShortToFloat(ts.eulerZ);

			// Print the newly populated struct data
			GD.Print($"--- Tank Status Update ---");
			GD.Print($"Drive: Left={ts.driveLeft}, Right={ts.driveRight}");
			GD.Print($"Euler: X={decodedEulerX}, Y={decodedEulerY}, Z={decodedEulerZ}");
			GD.Print($"Flag:  {ts.changeFlag}");
		}
	}
}
