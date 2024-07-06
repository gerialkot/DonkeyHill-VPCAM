# DonkeyHill-VPCAM

Giroszkóp forgásinformációk továbbítása ESP32-ről Unreal Engine-be Egy MPU6050 és FreeD Livelink Protokol segítségével.

Szükséges hardver:

- ESP32
- MPU6050

Bekötés:

vcc - ESP32 3v3
gnd - ESP32 gnd
SDA - ESP32 GPIO 15
SCL - ESP32 GPIO 22

A kódban meg kell határozni az alábbiakat:

- SSID: a wifi hálózat neve
- PASSWORD: A wifi hálózat jelszava
- PC_IP: a célszámítógép ip-címe
- PC_PORT: Az unreal engine Livelink komponensen belül beállított port (alapértelmezetten 8888)

Használat:

- USB-n csatlakoztassuk az ESP32-t, majd az Aruino Ide program segítségével töltsük fel a megadott kódot. Figyelj oda hogy a szükséges Library-k telepítve legyenek!
- Serial monitorban a program kiírja az ESP számára kiosztott ip-címet
- Unreal Engine projektünkben engedélyezzük a Livelink és LiveLinkFreeD Pluginokat, az EDIT>Pluigins menüpont alatt
- Nyissuk meg a Livelink ablakot a Window>Virtual Production>Livelink menüpont alatt
- A Livelink ablakban az "Add source gombra kattintva adjunk hozzá egy LiveLinkFreeD Source komponenst, a 0.0.0.0 ip-cím és a korábban beállított port megadásával.
- adjunk hozzá egy cinecamera actort a jelenetünkhöz.
- A Kamera "Details" füle alatt az "Add" hozzáadásával adjunk hozzá egy LiveLinkComponentController modult majd jelöljük ki azt a listán.
- A "Subject Representation" legördülő menüben adjuk hozzá a Camera 0-t
- számítógépünkön a böngészőbe írjuk be a korábban kapott ESP32-höz tartozó IP-címet, ami egy konfigurációs oldalt nyit meg, melyben állíthatjuk, hogy melyik tengely adatait akarjuk elküldeni livelinken, valamint hogy mekkora zajszűrést alkalmazunk a leolvasott tengelyadatokon.


