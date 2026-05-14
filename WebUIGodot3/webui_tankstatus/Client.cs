using System;
using Godot;

public class Client : Node
{
	private WebSocketClient _ws;
	private string _wsUrl = "ws://127.0.0.1:8881";

	private WebSocketClient _chatWs;
	private string _chatWsUrl = "wss://tankstatus.jordanirgangportfolio.com/ws/chat";

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

	// Wheels
	private CSGCylinder _leftWheel;
	private CSGCylinder _rightWheel;

	// Camera state
	private Camera _camera;
	private Spatial _cameraPivot;
	private bool _isDragging = false;
	private float _camRotationX = -Mathf.Pi / 4f; // Start looking down at a 45 degree angle
	private float _camRotationY = 0f;
	private float _mouseSensitivity = 0.005f;

	// Chat state
	private RichTextLabel _chatHistory;
	private RichTextLabel _replyBox;
	private LineEdit _chatInput;

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

		// --- Make the bot look like a 2-wheel stabilizer ---
		SpatialMaterial baseMaterial = new SpatialMaterial();
		baseMaterial.AlbedoColor = new Color(0.2f, 0.4f, 0.2f); // Dark Green Base
		puppetBase.Material = baseMaterial;
		
		SpatialMaterial platformMaterial = new SpatialMaterial();
		platformMaterial.AlbedoColor = new Color(0.7f, 0.7f, 0.8f); // Silver/Grey Stabilizer
		puppetPlatform.Material = platformMaterial;

		SpatialMaterial wheelMaterial = new SpatialMaterial();
		wheelMaterial.AlbedoColor = new Color(0.1f, 0.1f, 0.1f); // Dark Grey/Black

		SpatialMaterial spokeMaterial = new SpatialMaterial();
		spokeMaterial.AlbedoColor = new Color(0.8f, 0.2f, 0.2f); // Red spokes to see rotation

		// Left Wheel
		_leftWheel = new CSGCylinder();
		_leftWheel.Radius = puppetBase.Depth / 1.5f; // Large wheel
		_leftWheel.Height = 0.8f;
		_leftWheel.RotationDegrees = new Vector3(0, 0, 90);
		_leftWheel.Translation = new Vector3(-puppetBase.Width / 2 - 0.4f, 0, 0);
		_leftWheel.Material = wheelMaterial;
		puppetBase.AddChild(_leftWheel);

		CSGBox leftSpoke = new CSGBox();
		leftSpoke.Width = _leftWheel.Radius * 1.8f;
		leftSpoke.Height = 0.9f; // Slightly wider than wheel thickness so it's visible
		leftSpoke.Depth = 0.2f;
		leftSpoke.Material = spokeMaterial;
		_leftWheel.AddChild(leftSpoke);

		// Right Wheel
		_rightWheel = new CSGCylinder();
		_rightWheel.Radius = puppetBase.Depth / 1.5f;
		_rightWheel.Height = 0.8f;
		_rightWheel.RotationDegrees = new Vector3(0, 0, 90);
		_rightWheel.Translation = new Vector3(puppetBase.Width / 2 + 0.4f, 0, 0);
		_rightWheel.Material = wheelMaterial;
		puppetBase.AddChild(_rightWheel);

		CSGBox rightSpoke = new CSGBox();
		rightSpoke.Width = _rightWheel.Radius * 1.8f;
		rightSpoke.Height = 0.9f;
		rightSpoke.Depth = 0.2f;
		rightSpoke.Material = spokeMaterial;
		_rightWheel.AddChild(rightSpoke);
		// --------------------------------------

		// --- Setup Camera Pivot ---
		_camera = GetNode<Camera>("Camera");
		
		_cameraPivot = new Spatial();
		_cameraPivot.Translation = puppetBase.Translation; // Center on the tank initially
		AddChild(_cameraPivot);

		// Remove camera from 'Node' and make it a child of the pivot
		RemoveChild(_camera);
		_cameraPivot.AddChild(_camera);
		
		// Set the camera's local offset (move back and slightly up from pivot)
		_camera.Transform = new Transform(Basis.Identity, new Vector3(0, 5, 12)); // Reset to 5 up, 12 back
		
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

		_chatWs = new WebSocketClient();
		_chatWs.Connect("connection_closed", this, nameof(OnChatConnectionClosed));
		_chatWs.Connect("connection_error", this, nameof(OnChatConnectionError));
		_chatWs.Connect("connection_established", this, nameof(OnChatConnectionEstablished));
		_chatWs.Connect("data_received", this, nameof(OnChatDataReceived));

		Error chatErr = _chatWs.ConnectToUrl(_chatWsUrl);
		if (chatErr == Error.Ok)
		{
			GD.Print($"Connecting to Chat WebSocket at {_chatWsUrl}...");
		}
		else
		{
			GD.PrintErr($"Failed to start Chat WebSocket client. Error code: {chatErr}");
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

		// Chat Space UI
		VBoxContainer chatContainer = new VBoxContainer();
		chatContainer.AnchorLeft = 1;
		chatContainer.AnchorRight = 1;
		chatContainer.AnchorBottom = 1;
		chatContainer.MarginLeft = -320;
		chatContainer.MarginTop = -220; 
		chatContainer.MarginRight = -20;
		chatContainer.MarginBottom = -20;
		canvasLayer.AddChild(chatContainer);

		_chatHistory = new RichTextLabel();
		_chatHistory.SizeFlagsVertical = (int)Control.SizeFlags.ExpandFill;
		_chatHistory.ScrollActive = true;
		_chatHistory.BbcodeEnabled = true;
		_chatHistory.BbcodeText = "[color=yellow]Chat Space Initialized[/color]\n";
		chatContainer.AddChild(_chatHistory);

		_replyBox = new RichTextLabel();
		_replyBox.RectMinSize = new Vector2(0, 60); // Give it some height
		_replyBox.BbcodeEnabled = true;
		_replyBox.BbcodeText = "[color=cyan]Replies will appear here...[/color]";
		chatContainer.AddChild(_replyBox);

		HBoxContainer chatInputBox = new HBoxContainer();
		chatContainer.AddChild(chatInputBox);

		_chatInput = new LineEdit();
		_chatInput.SizeFlagsHorizontal = (int)Control.SizeFlags.ExpandFill;
		_chatInput.PlaceholderText = "Type message...";
		_chatInput.Connect("text_entered", this, nameof(OnChatEntered));
		chatInputBox.AddChild(_chatInput);

		Button btnSendChat = new Button();
		btnSendChat.Text = "Send";
		btnSendChat.Connect("pressed", this, nameof(OnBtnSendChatPressed));
		chatInputBox.AddChild(btnSendChat);
	}

	public override void _Process(float delta)
	{
		if (_ws != null && _ws.GetConnectionStatus() != NetworkedMultiplayerPeer.ConnectionStatus.Disconnected)
		{
			_ws.Poll();
		}

		if (_chatWs != null && _chatWs.GetConnectionStatus() != NetworkedMultiplayerPeer.ConnectionStatus.Disconnected)
		{
			_chatWs.Poll();
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

		// Rotate the wheels based on motor speed
		if (_leftWheel != null)
		{
			// local Y axis is the wheel's spin axis because it's rotated 90 on Z
			_leftWheel.RotateY(-currentLeftMotor * 15.0f * delta); 
		}
		if (_rightWheel != null)
		{
			_rightWheel.RotateY(-currentRightMotor * 15.0f * delta);
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
		// Grab the raw bytes and convert to string
		byte[] packet = _ws.GetPeer(1).GetPacket();
		string jsonStr = System.Text.Encoding.UTF8.GetString(packet);

		JSONParseResult result = JSON.Parse(jsonStr);
		if (result.Error != Error.Ok)
		{
			GD.PrintErr("Failed to parse telemetry JSON");
			return;
		}

		Godot.Collections.Dictionary data = result.Result as Godot.Collections.Dictionary;
		
		decodedEulerX = Convert.ToSingle(data["eulerX"]);
		decodedEulerY = Convert.ToSingle(data["eulerY"]);
		decodedEulerZ = Convert.ToSingle(data["eulerZ"]);
		
		byte driveLeft = Convert.ToByte(data["driveLeft"]);
		byte driveRight = Convert.ToByte(data["driveRight"]);

		currentLeftMotor = (driveLeft - 127) / 128.0f;
		currentRightMotor = (driveRight - 127) / 128.0f;

		// Update UI
		lblDriveLeft.Text = $"Drive Left: {driveLeft}";
		lblDriveRight.Text = $"Drive Right: {driveRight}";
		lblEuler.Text = $"Euler: X={decodedEulerX:F2}, Y={decodedEulerY:F2}, Z={decodedEulerZ:F2}";
		float forwardSpeed = (currentLeftMotor + currentRightMotor) / 2.0f;
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

		string jsonCmd = $"{{\"driveLeft\": {left}, \"driveRight\": {right}, \"changeFlag\": 1}}";
		_ws.GetPeer(1).PutPacket(System.Text.Encoding.UTF8.GetBytes(jsonCmd));
		GD.Print($"Sent JSON command: {jsonCmd}");
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

	private void OnBtnSendChatPressed()
	{
		OnChatEntered(_chatInput.Text);
	}

	private void OnChatEntered(string newText)
	{
		if (string.IsNullOrEmpty(newText)) return;
		
		_chatHistory.BbcodeText += $"\n[color=lightblue]You:[/color] {newText}";
		_chatInput.Text = "";
		
		if (_chatWs != null && _chatWs.GetConnectionStatus() == NetworkedMultiplayerPeer.ConnectionStatus.Connected)
		{
			string jsonMsg = $"{{\"message\": \"{newText}\"}}";
			_chatWs.GetPeer(1).PutPacket(System.Text.Encoding.UTF8.GetBytes(jsonMsg));
		}
	}
	private void OnChatConnectionEstablished(string protocol)
	{
		GD.Print($"Chat WebSocket connection established with protocol: {protocol}");
		_chatHistory.BbcodeText += "\n[color=green]Chat connected![/color]";
	}

	private void OnChatConnectionClosed(bool wasClean)
	{
		GD.Print($"Chat WebSocket connection closed, clean: {wasClean}");
		_chatHistory.BbcodeText += "\n[color=red]Chat disconnected![/color]";
	}

	private void OnChatConnectionError()
	{
		GD.Print("Chat WebSocket connection error");
		_chatHistory.BbcodeText += "\n[color=red]Chat connection error![/color]";
	}

	private void OnChatDataReceived()
	{
		byte[] packet = _chatWs.GetPeer(1).GetPacket();
		string jsonStr = System.Text.Encoding.UTF8.GetString(packet);
		
		JSONParseResult result = JSON.Parse(jsonStr);
		if (result.Error != Error.Ok)
		{
			// Fallback if the server sends plain text instead of JSON
			_chatHistory.BbcodeText += $"\n[color=lightgreen]Server:[/color] {jsonStr}";
			_replyBox.BbcodeText = $"[color=lightgreen]Latest Reply:[/color] {jsonStr}";
			return;
		}

		Godot.Collections.Dictionary data = result.Result as Godot.Collections.Dictionary;
		string type = data.Contains("type") ? data["type"].ToString() : "";
		
		if (type == "chunk")
		{
			string content = data["content"].ToString();
			_chatHistory.BbcodeText += content; // Streaming effect
			_replyBox.BbcodeText = $"[color=lightgreen]Latest:[/color] {content}";
		}
		else if (type == "done")
		{
			_chatHistory.BbcodeText += "\n";
		}
		else if (type == "error")
		{
			string errorMsg = data["content"].ToString();
			_chatHistory.BbcodeText += $"\n[color=red]Error:[/color] {errorMsg}";
		}
	}
}
