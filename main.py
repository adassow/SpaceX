from main.ota_updater import OTAUpdater
from main import wifimgr
from main.main import SpaceX

def download_and_install_update_if_available():
    o = OTAUpdater('https://github.com/adassow/SpaceX')
    o.download_and_install_update_if_available('', '')


def start():
    
    project = SpaceX()

def boot():
    wlan = wifimgr.get_connection()
    if wlan is None:    
        print("Could not initialize the network connection.")
        while True:
            pass  # you shall not pass :D
    download_and_install_update_if_available()
    start()


boot()