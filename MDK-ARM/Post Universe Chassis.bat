@echo off
C:\Keil_v5\ARM\ARMCC\bin\fromelf.exe --bin --output "bin/Chassis_xx_xx_xx.bin" "Chasis_Project/Chasis_Project.axf"
CalculateFirmwareCRC 16 3976 "bin\Chassis_xx_xx_xx.bin"