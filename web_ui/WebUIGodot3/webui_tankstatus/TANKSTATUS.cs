using System;
using System.Runtime.InteropServices;
using Godot;

public class TANKSTATUS : Node
{
	TankStatus ts;

	// Called when the node enters the scene tree for the first time.
	public override void _Ready()
	{
		ts = new TankStatus();
		TankStatusNative.constructTankStatus(ref ts);
	}

	//  // Called every frame. 'delta' is the elapsed time since the previous frame.
	//  public override void _Process(float delta)
	//  {
	//
	//  }
}
