# TFG-Jaume-Sudria-Roura

Aquest repositori conté el codi i fitxers relacionats amb el desenvolupament del Treball de Final de Grau: Desenvolupament d'un dispositiu per monitorar les compressions toràciques en la reanimació cardiopulmonar.

Estructura del repositori:
/
├── zephyr_firmware/ # Codi principal Zephyr
│ ├── main.c # Fil principal i control de flux
│ ├── remote.h # Definicions compartides
│ ├── remote.c # Gestió de notificacions i estat
│ ├── CMakeLists.txt # Build script del projecte
│ ├── power_config.overlay # Device Tree overlay
│ └── prj.conf # Configuració principal Zephyr
│
├── board_configuration/ # Fitxers per a la definició de placa
│ ├── board.cmake # Build script per a la placa
│ ├── board.yml   # Metadata de la placa
│ ├── cpr_board_v3.dts # Device Tree Source
│ ├── cpr_board_v3.yaml # Compatibilitat i descripció
│ ├── cpr_board_v3_defconfig # Configuració per defecte
│ ├── cpr_board_v3-pinctrl.dtsi # Assignació de pins
│ ├── Kconfig.cpr_board_v3 # Opcions personalitzades Kconfig
│ ├── Kconfig.defconfig # Configs per defecte
│ └── pre_dt_board.cmake # Script pre-Device Tree
│
├── Fitxers stl/ # Models 3D de la carcassa i simulació
│ ├── Encapsulat/
│ │ ├── Encapsulat-inferior.stl
│ │ ├── Encapsulat-interior.stl
│ │ └── Encapsulat-superior.stl
│ └── Simulació/
│ ├── simulacio-base-0graus.stl
│ ├── simulacio-base-20graus.stl
│ └── simulacio-guia.stl
│
├── Gerber PCB/ # Fitxers per fabricar la PCB rígida
│ └── Gerber_TFG-placa-final.zip
│
├── Gerber FPCB/ # Fitxers per fabricar la PCB flexible
│ └── Gerber_FPCB.zip
