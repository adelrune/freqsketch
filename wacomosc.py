from pythonosc import udp_client
from subprocess import Popen, PIPE

osc = udp_client.SimpleUDPClient("127.0.0.1", 5005)
libin = Popen(["unbuffer", "libinput", "debug-events"], stdout=PIPE)
while True:
    l = libin.stdout.readline().decode("utf-8")

    # tilt 3
    # pressure 4
    try:
        if "TABLET_TOOL" in l:
            osc.send_message("/tablet/tilt", [float(num.strip()) for num in l.split("\t")[3].split("tilt:")[1].replace("*","").split("/")])
            if "pressure" in l:
                osc.send_message("/tablet/pressure", float(l.split("\t")[4].split("pressure:")[1].split(" ")[1].replace("*","")))
    except:
        pass
