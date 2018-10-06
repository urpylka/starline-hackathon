cat << EOF > /lib/systemd/system/turtle_express.service
[Unit]
Description=Turtle Express service
Requires=roscore.service
After=roscore.service

[Service]
EnvironmentFile=/root/roscore.env
ExecStart=/opt/ros/kinetic/bin/roslaunch turtle_express turtle_express.launch --wait
Restart=on-abort

[Install]
WantedBy=multi-user.target
EOF