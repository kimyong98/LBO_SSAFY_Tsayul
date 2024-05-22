@echo off
call C:\dev\ros2-eloquent\setup.bat
colcon build
call install\local_setup.bat
