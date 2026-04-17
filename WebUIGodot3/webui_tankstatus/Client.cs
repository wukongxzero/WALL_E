using System;
using Godot;

public class Client : Node
{
	private PacketPeerUDP _udp;
	private int _port = 8881;

	public override void _Ready()
	{
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
			
			// If we received exactly 4 bytes, it's likely the Python float
			if (packet.Length == 4)
			{
				float receivedFloat = BitConverter.ToSingle(packet, 0);
				GD.Print($"Received Float: {receivedFloat}");
			}
			else
			{
				// Fallback: Try to print it as a normal string
				string rawText = System.Text.Encoding.UTF8.GetString(packet);
				GD.Print($"Received String/Raw: {rawText}");
			}
		}
	}
}
