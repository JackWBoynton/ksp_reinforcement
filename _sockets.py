
import socket
import struct
import time, sys, re

# Telnet protocol characters (don't change)
IAC  = bytes([255]) # "Interpret As Command"
DONT = bytes([254])
DO   = bytes([253])
WONT = bytes([252])
WILL = bytes([251])
theNULL = bytes([0])

SE  = bytes([240])  # Subnegotiation End
NOP = bytes([241])  # No Opseration
DM  = bytes([242])  # Data Mark
BRK = bytes([243])  # Break
IP  = bytes([244])  # Interrupt process
AO  = bytes([245])  # Abort output
AYT = bytes([246])  # Are You There
EC  = bytes([247])  # Erase Character
EL  = bytes([248])  # Erase Line
GA  = bytes([249])  # Go Ahead
SB =  bytes([250])  # Subnegotiation Begin


# Telnet protocol options code (don't change)
# These ones all come from arpa/telnet.h
BINARY = bytes([0]) # 8-bit data path
ECHO = bytes([1]) # echo
RCP = bytes([2]) # prepare to reconnect
SGA = bytes([3]) # suppress go ahead
NAMS = bytes([4]) # approximate message size
STATUS = bytes([5]) # give status
TM = bytes([6]) # timing mark
RCTE = bytes([7]) # remote controlled transmission and echo
NAOL = bytes([8]) # negotiate about output line width
NAOP = bytes([9]) # negotiate about output page size
NAOCRD = bytes([10]) # negotiate about CR disposition
NAOHTS = bytes([11]) # negotiate about horizontal tabstops
NAOHTD = bytes([12]) # negotiate about horizontal tab disposition
NAOFFD = bytes([13]) # negotiate about formfeed disposition
NAOVTS = bytes([14]) # negotiate about vertical tab stops
NAOVTD = bytes([15]) # negotiate about vertical tab disposition
NAOLFD = bytes([16]) # negotiate about output LF disposition
XASCII = bytes([17]) # extended ascii character set
LOGOUT = bytes([18]) # force logout
BM = bytes([19]) # byte macro
DET = bytes([20]) # data entry terminal
SUPDUP = bytes([21]) # supdup protocol
SUPDUPOUTPUT = bytes([22]) # supdup output
SNDLOC = bytes([23]) # send location
TTYPE = bytes([24]) # terminal type
EOR = bytes([25]) # end or record
TUID = bytes([26]) # TACACS user identification
OUTMRK = bytes([27]) # output marking
TTYLOC = bytes([28]) # terminal location number
VT3270REGIME = bytes([29]) # 3270 regime
X3PAD = bytes([30]) # X.3 PAD
NAWS = bytes([31]) # window size
TSPEED = bytes([32]) # terminal speed
LFLOW = bytes([33]) # remote flow control
LINEMODE = bytes([34]) # Linemode option
XDISPLOC = bytes([35]) # X Display Location
OLD_ENVIRON = bytes([36]) # Old - Environment variables
AUTHENTICATION = bytes([37]) # Authenticate
ENCRYPT = bytes([38]) # Encryption option
NEW_ENVIRON = bytes([39]) # New - Environment variables
# the following ones come from
# http://www.iana.org/assignments/telnet-options
# Unfortunately, that document does not assign identifiers
# to all of them, so we are making them up
TN3270E = bytes([40]) # TN3270E
XAUTH = bytes([41]) # XAUTH
CHARSET = bytes([42]) # CHARSET
RSP = bytes([43]) # Telnet Remote Serial Port
COM_PORT_OPTION = bytes([44]) # Com Port Control Option
SUPPRESS_LOCAL_ECHO = bytes([45]) # Telnet Suppress Local Echo
TLS = bytes([46]) # Telnet Start TLS
KERMIT = bytes([47]) # KERMIT
SEND_URL = bytes([48]) # SEND-URL
FORWARD_X = bytes([49]) # FORWARD_X
PRAGMA_LOGON = bytes([138]) # TELOPT PRAGMA LOGON
SSPI_LOGON = bytes([139]) # TELOPT SSPI LOGON
PRAGMA_HEARTBEAT = bytes([140]) # TELOPT PRAGMA HEARTBEAT
EXOPL = bytes([255]) # Extended-Options-List
NOOPT = bytes([0])

class KSPController(object): # object that controls the connection to the kOS terminal in KSP
    def __init__(self):
        self.conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.conn.connect(("127.0.0.1", 1024))
        self.conn.setblocking(False)
        width = struct.pack("H", 47)
        height = struct.pack("H", 243)
        self.conn.sendall(IAC + WILL + NAWS)
        self.conn.sendall(IAC + SB + NAWS + width + height + IAC + SE) # set term size
        time.sleep(1)
        self.conn.sendall(IAC + WILL + TTYPE)
        self.conn.sendall(IAC + SB + TTYPE + b'\0' + b'XTERM-256COLOR' + IAC + SE) # set term type
        time.sleep(10)

    def establish_cpu(self):
        # method that connects to telnet kOS port and streams telemetry data through serialized data structure
         # send the cpu selection and enter to connect to the proper terminal
        out = self.conn.recv(4096)
        print(out)
        if b"Connected to the kOS Terminal Server" not in out:
            sys.exit(0)
        time.sleep(2)
        
       
        time.sleep(1)
        #self.conn.sendall(b"\x31\x0d\x13") # 1 + enter
        #self.conn.sendall(b"\x31\x0d\x0a") # 1 + enter
        self.conn.sendall(b"\x31\x0d\x0a") # 1 + enter
        self.conn.sendall(b"\x31\x0d\x0a")
        #out = self.conn.recv(1000)
        print(out)
        while True:
            #self.conn.sendall(b"\x31\x0d\x0a") # 1 + enter
            #self.conn.sendall(b"\x70\x72\x69\x6e\x74\x20\x61\x6c\x74\x3a\x72\x61\x64\x61\x72\x2e\x0d\x0a")
            time.sleep(1)
            out = self.conn.recv(32768)
            out = list(filter(lambda x: x != b"_" and x != 32 and x != 45, out))
            #out = b'|'.join(re.escape(bytes(x)) for x in out)
            out = ''.join(map(chr,out))
            print(out)
            time.sleep(5)

    def get_telemetry(self):
        pass
    

if __name__ == "__main__":
    conn = KSPController()
    conn.establish_cpu()