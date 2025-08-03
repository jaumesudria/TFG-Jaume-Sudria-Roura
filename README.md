# TFG-Jaume-Sudria-Roura

Aquest repositori conté el codi i fitxers relacionats amb el desenvolupament del Treball de Final de Grau: Desenvolupament d'un dispositiu per monitorar les compressions toràciques en la reanimació cardiopulmonar.

Estructura del repositori:

- Zephyr project/
    - main.c
    - remote.h
    - remote.c
    - CMakeList.txt
    - power_config.overlay
    - prj.conf
 
- Board configuration/
    - board.cmake
    - board.yml
    - cpr_board_v3.dts
    - cpr_board_v3.yaml
    - cpr_board_v3_defconfig
    - cpr_board_v3-pinctrl.dtsi
    - Kconfig.cpr_board_v3
    - Kconfig.defconfig
    - pre_dt_board.cmake
    
- Fitxers stl/
    - Encapsulat/
         - Encapsulat-inferior.stl
         - Encapsulat-interior.stl
         - Encapsulat-superior.stl
    - Simulació/
        -  simulacio-base-0graus.stl
        -  simulacio-base-20graus.stl
        -  simulacio-guia.stl 

- Gerber PCB/                  
    - Gerber_TFG-placa-final.zip

- Gerber PCB/                   
    - Gerber_FPCB.zip
