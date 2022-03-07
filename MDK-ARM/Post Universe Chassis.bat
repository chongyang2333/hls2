@echo off
fromelf --bin --output "bin/Chassis0D_01-xx_xx_xx.bin" "Chasis_Project/Chasis_Project.axf"
CalculateFirmwareCRC 16 3976 "bin\Chassis0D_01-xx_xx_xx.bin"