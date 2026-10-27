@echo off
chcp 65001 > nul
title Simulation Chirurgicale d'Incision - Etape 5 (Premiere Coupe) - SOFA
echo ============================================================================
echo   LANCEMENT DE LA SIMULATION D'INCISION DE LA PEAU (ETAPE 5 : PREMIERE COUPE)
echo ============================================================================
"C:\Users\djaou\SOFA\v26.06.00\bin\runSofa.exe" "%~dp0skin_simulation.py"
pause
