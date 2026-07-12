# systemctl / systemd — Notes

`systemd` is the service manager most Linux distros (including Ubuntu, what
the Jetson and this dev machine run) use to start, stop, and supervise
background processes ("services" / "units"). `systemctl` is the command-line
tool for controlling it.

## Core commands

```bash
sudo systemctl start <service>      # start now
sudo systemctl stop <service>       # stop now
sudo systemctl restart <service>    # stop then start
sudo systemctl status <service>     # is it running? recent log lines?
sudo systemctl enable <service>     # start automatically on boot
sudo systemctl disable <service>    # don't start automatically on boot
sudo systemctl enable --now <service>  # enable AND start in one shot
```

`enable` vs `start` is the thing people mix up: `start` runs it *right now*;
`enable` makes it run *on every future boot*. They're independent — you can
`start` without `enable` (runs now, won't survive a reboot) or `enable`
without `start` (will run on next boot, not running right now).

## Checking logs

Services managed by systemd don't dump to a regular file by default — their
output goes to the journal:

```bash
journalctl -u <service>             # all logs for this service
journalctl -u <service> -f          # follow live, like tail -f
journalctl -u <service> --since "10 min ago"
```

## Writing a unit file

Unit files live in `/etc/systemd/system/<name>.service`. Minimal example:

```ini
[Unit]
Description=My service
After=network.target

[Service]
ExecStart=/path/to/program --args
Restart=on-failure
User=someuser

[Install]
WantedBy=multi-user.target
```

After creating/editing a unit file:
```bash
sudo systemctl daemon-reload   # tell systemd to re-read unit files
sudo systemctl enable --now my-service
```

## Where this fits WALL-E

The README targets a Jetson Orin Nano as the onboard compute — a robot that's
meant to boot up and run autonomously, not be launched by hand from a
terminal every time. Right now nothing in this repo auto-starts; you'd
`ros2 launch wall_e_bringup wall_e.launch.py` manually each time.

A systemd unit wrapping that launch command would fix that — e.g.:

```ini
[Unit]
Description=WALL-E ROS2 bringup
After=network.target

[Service]
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && source /home/<user>/WALL_E/install/setup.bash && ros2 launch wall_e_bringup wall_e.launch.py"
Restart=on-failure
User=<user>

[Install]
WantedBy=multi-user.target
```

**Things to actually think through before wiring this up** (not yet done):
- Serial devices (`/dev/ttyUSB0`, `/dev/ttyUSB1`) may not be present/ready
  immediately at boot — might need a udev rule or a startup delay/retry
  rather than assuming they're there the instant the service starts.
- `Restart=on-failure` means a crash loop is possible if something's
  wrong at boot (e.g. Arduino not plugged in yet) — worth pairing with
  `RestartSec=` and maybe `StartLimitBurst=` to avoid hammering it.
- This should probably wrap `mega_node`/hardware-facing nodes specifically,
  not the whole stack, so a bad boot doesn't also take down anything you
  might want to debug interactively over SSH.

**Status:** notes only, no unit file created yet.
