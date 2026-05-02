using System;
using Godot;

public class Client : Node
{
	private WebSocketClient _ws;
	private string _wsUrl = "ws://127.0.0.1:8881";
	TankStatus localTankSubscriber;

	CSGBox puppetPlatform;
	CSGBox puppetBase;

	float decodedEulerX;
	float decodedEulerY;
	float decodedEulerZ;

	float currentLeftMotor;
	float currentRightMotor;

	private Label lblDriveLeft;
	private Label lblDriveRight;
	private Label lblEuler;
	private Label lblSpeed;

	// Camera state
	private Camera _camera;
	private Spatial _cameraPivot;
	private bool _isDragging = false;
	private float _camRotationX = -Mathf.Pi / 4f; // Start looking down at a 45 degree angle
	private float _camRotationY = 0f;
	private float _mouseSensitivity = 0.005f;

	public override void _Ready()
	{
		TankStatusNative.constructTankStatus(ref localTankSubscriber);
		_ws = new WebSocketClient();
		_ws.Connect("connection_closed", this, nameof(OnConnectionClosed));
		_ws.Connect("connection_error", this, nameof(OnConnectionError));
		_ws.Connect("connection_established", this, nameof(OnConnectionEstablished));
		_ws.Connect("data_received", this, nameof(OnDataReceived));

		puppetBase = GetNode<CSGBox>("BaseCSGBox");
		puppetPlatform = GetNode<CSGBox>("BaseCSGBox").GetNode<CSGBox>("CSGPlatform");

		// --- Setup Camera Pivot ---
		_camera = GetNode<Camera>("Camera");
		
		_cameraPivot = new Spatial();
		_cameraPivot.Translation = puppetBase.Translation; // Center on the tank initially
		AddChild(_cameraPivot);

		// Remove camera from 'Node' and make it a child of the pivot
		RemoveChild(_camera);
		_cameraPivot.AddChild(_camera);
		
		// Set the camera's local offset (move back and slightly up from pivot)
		_camera.Transform = new Transform(Basis.Identity, new Vector3(0, 2, 12));
		
		// Apply initial rotations
		_cameraPivot.Rotation = new Vector3(_camRotationX, _camRotationY, 0);
		_camera.LookAt(_cameraPivot.GlobalTransform.origin, Vector3.Up);
		// --------------------------

		Error err = _ws.ConnectToUrl(_wsUrl);

		if (err == Error.Ok)
		{
			GD.Print($"Connecting to WebSocket at {_wsUrl}...");
		}
		else
		{
			GD.PrintErr($"Failed to start WebSocket client. Error code: {err}");
		}

		// Initialize UI
		CanvasLayer canvasLayer = new CanvasLayer();
		AddChild(canvasLayer);

		VBoxContainer vbox = new VBoxContainer();
		vbox.MarginLeft = 20;
		vbox.MarginTop = 20;
		canvasLayer.AddChild(vbox);

		lblDriveLeft = new Label();
		lblDriveLeft.Text = "Drive Left: ";
		vbox.AddChild(lblDriveLeft);

		lblDriveRight = new Label();
		lblDriveRight.Text = "Drive Right: ";
		vbox.AddChild(lblDriveRight);

		lblEuler = new Label();
		lblEuler.Text = "Euler: ";
		vbox.AddChild(lblEuler);

		lblSpeed = new Label();
		lblSpeed.Text = "Speed: ";
		vbox.AddChild(lblSpeed);
		
		// Control Buttons
		HBoxContainer hboxControls = new HBoxContainer();
		vbox.AddChild(hboxControls);

		Button btnForward = new Button();
		btnForward.Text = "Forward";
		btnForward.Connect("pressed", this, nameof(OnBtnForwardPressed));
		hboxControls.AddChild(btnForward);

		Button btnBackward = new Button();
		btnBackward.Text = "Backward";
		btnBackward.Connect("pressed", this, nameof(OnBtnBackwardPressed));
		hboxControls.AddChild(btnBackward);

		Button btnLeft = new Button();
		btnLeft.Text = "Left";
		btnLeft.Connect("pressed", this, nameof(OnBtnLeftPressed));
		hboxControls.AddChild(btnLeft);

		Button btnRight = new Button();
		btnRight.Text = "Right";
		btnRight.Connect("pressed", this, nameof(OnBtnRightPressed));
		hboxControls.AddChild(btnRight);

		Button btnStop = new Button();
		btnStop.Text = "Stop";
		btnStop.Connect("pressed", this, nameof(OnBtnStopPressed));
		hboxControls.AddChild(btnStop);
	}

	public override void _Process(float delta)
	{
		if (_ws != null && _ws.GetConnectionStatus() != NetworkedMultiplayerPeer.ConnectionStatus.Disconnected)
		{
			_ws.Poll();
		}

		float turnSpeed = (currentLeftMotor - currentRightMotor);
		float rotationMultiplier = 2.0f; // Adjust turn sensitivity
		puppetBase.RotateY(-turnSpeed * rotationMultiplier * delta);
		
		float forwardSpeed = (currentLeftMotor + currentRightMotor) / 2.0f;
		puppetBase.Translate(new Vector3(0, 0, -forwardSpeed * 5.0f * delta));

		// Follow the tank with the camera pivot
		if (_cameraPivot != null && puppetBase != null)
		{
			_cameraPivot.Translation = puppetBase.Translation;
		}

		puppetPlatform.RotationDegrees = new Vector3(
			decodedEulerX,
			decodedEulerY,
			decodedEulerZ
		);
	}

	public override void _UnhandledInput(InputEvent @event)
	{
		if (@event is InputEventMouseButton mouseBtn)
		{
			if (mouseBtn.ButtonIndex == (int)ButtonList.Left)
			{
				_isDragging = mouseBtn.Pressed;
			}
		}
		else if (@event is InputEventMouseMotion mouseMotion && _isDragging)
		{
			_camRotationY -= mouseMotion.Relative.x * _mouseSensitivity;
			_camRotationX -= mouseMotion.Relative.y * _mouseSensitivity;

			// Clamp X rotation (pitch) so the camera doesn't flip completely upside down
			_camRotationX = Mathf.Clamp(_camRotationX, -Mathf.Pi / 2.2f, Mathf.Pi / 2.2f);

			_cameraPivot.Rotation = new Vector3(_camRotationX, _camRotationY, 0);
		}
	}

	private void OnConnectionEstablished(string protocol)
	{
		GD.Print($"WebSocket connection established with protocol: {protocol}");
	}

	private void OnConnectionClosed(bool wasClean)
	{
		GD.Print($"WebSocket connection closed, clean: {wasClean}");
	}

	private void OnConnectionError()
	{
		GD.Print("WebSocket connection error");
	}

	private void OnDataReceived()
	{
		// Grab the raw bytes
		byte[] packet = _ws.GetPeer(1).GetPacket();

		// Initialize a fresh struct
		TankStatus ts = new TankStatus();

		// Optional: Call your C constructor if it sets default values
		TankStatusNative.constructTankStatus(ref ts);

		// Pass the byte array and the struct reference to the C library
		TankStatusNative.readByteTankStatus(packet, packet.Length, ref ts);
		decodedEulerX = TankStatusNative.getDecodedShortToFloat(ts.eulerX);
		decodedEulerY = TankStatusNative.getDecodedShortToFloat(ts.eulerY);
		decodedEulerZ = TankStatusNative.getDecodedShortToFloat(ts.eulerZ);
		
		currentLeftMotor = (ts.driveLeft - 127) / 128.0f;
		currentRightMotor = (ts.driveRight - 127) / 128.0f;

		// Print the newly populated struct data
		GD.Print($"--- Tank Status Update ---");
		GD.Print($"Drive: Left={ts.driveLeft}, Right={ts.driveRight}");
		GD.Print($"Euler: X={decodedEulerX}, Y={decodedEulerY}, Z={decodedEulerZ}");
		GD.Print($"Flag:  {ts.changeFlag}");

		float forwardSpeed = (currentLeftMotor + currentRightMotor) / 2.0f;

		// Update UI
		lblDriveLeft.Text = $"Drive Left: {ts.driveLeft}";
		lblDriveRight.Text = $"Drive Right: {ts.driveRight}";
		lblEuler.Text = $"Euler: X={decodedEulerX:F2}, Y={decodedEulerY:F2}, Z={decodedEulerZ:F2}";
		lblSpeed.Text = $"Speed: {forwardSpeed:F2}";
		
		// Change color depending on speed
		if (forwardSpeed > 0.5f)
			lblSpeed.Modulate = new Color(0, 1, 0); // Green for fast forward
		else if (forwardSpeed > 0.1f)
			lblSpeed.Modulate = new Color(1, 1, 0); // Yellow for slow forward
		else if (forwardSpeed < -0.1f)
			lblSpeed.Modulate = new Color(1, 0, 0); // Red for backward
		else
			lblSpeed.Modulate = new Color(1, 1, 1); // White for stopped/idle
	}

	private void SendDriveCommand(byte left, byte right)
	{
		if (_ws == null || _ws.GetConnectionStatus() != NetworkedMultiplayerPeer.ConnectionStatus.Connected)
		{
			GD.Print("Cannot send command, WebSocket is not connected.");
			return;
		}

		TankStatus ts = new TankStatus();
		TankStatusNative.constructTankStatus(ref ts);
		ts.driveLeft = left;
		ts.driveRight = right;
		ts.changeFlag = 1;

		int packetLen = TankStatusNative.get_tankstatus_packet_length();
		byte[] buffer = new byte[packetLen];

		TankStatusNative.makeByteTankStatus(buffer, packetLen, ref ts);

		_ws.GetPeer(1).PutPacket(buffer);
		GD.Print($"Sent command: Left={left}, Right={right}");
	}

	private void OnBtnForwardPressed()
	{
		SendDriveCommand(255, 255);
	}

	private void OnBtnBackwardPressed()
	{
		SendDriveCommand(0, 0);
	}

	private void OnBtnLeftPressed()
	{
		SendDriveCommand(0, 255);
	}

	private void OnBtnRightPressed()
	{
		SendDriveCommand(255, 0);
	}

	private void OnBtnStopPressed()
	{
		SendDriveCommand(127, 127);
	}
}
