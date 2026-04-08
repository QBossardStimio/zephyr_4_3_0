# samples/bluetooth/peripheral_ots_railnet/CMakeLists.txt
#
# Firmware Zephyr OTS (Object Transfer Service) pour NINA-B301 / nRF52840
# sur la carte RailNet200.
#
# Ce firmware remplace hci_uart : le NINA-B301 n'est plus un contrôleur HCI
# mais un périphérique BLE autonome exposant un serveur GATT OTS (UUID 0x1825).
#
# Architecture :
#   Android (OTS Central) <-- BLE L2CAP CoC --> NINA-B301 <-- UART SOTP --> iMX6
#
# Voir docs/ble-obex-bredr-investigation.md pour le contexte complet.

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(peripheral_ots_railnet)

target_sources(app PRIVATE
    src/main.c
    src/ots_handler.c
    src/uart_relay.c
)
