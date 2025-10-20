import socket
import struct
import time

def get_optitrack_position():
    """Get current position from OptiTrack system."""
    # OptiTrack multicast settings
    multicast_address = "239.255.42.99"
    port = 1511

    # Create socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', port))

    # Join multicast group
    mreq = struct.pack("4sl", socket.inet_aton(multicast_address), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    sock.settimeout(1.0) #1 SECOND TIMEOUT
    
    try:
        while True:
            try:
                data, addr = sock.recvfrom(65507)  # Max UDP packet size
                if len(data) < 4:
                    continue

                # Parse message header
                message_id = int.from_bytes(data[0:2], byteorder='little')
                byte_count = int.from_bytes(data[2:4], byteorder='little')

                if message_id != 7:  # Not a motion data packet
                    continue
                offset = 4 #skip message ID and size

                #skip frame number
                offset += 4

                # skip marker set count and data
                marker_set_count = int.from_bytes(data[offset:offset+4], byteorder='little')
                offset += 4

                #get number of rigid bodies
                rigid_body_count = int.from_bytes(data[offset:offset+4], byteorder='little')
                offset += 4

                if rigid_body_count > 0:
                    # skip rigid body ID
                    rb_id = int.from_bytes(data[offset:offset+4], byteorder='little')
                    offset += 4

                    # get positition - Optitrack coordinatew system: +Y=up +X=left +Z=forward
                    x = struct.unpack('f', data[offset:offset+4])[0] #left+
                    y = struct.unpack('f', data[offset+4:offset+8])[0] #up+
                    z = struct.unpack('f', data[offset+8:offset+12])[0] #forward+

                    return x, y, z, rb_id
            except socket.timeout:
                    print("No OptiTrack data received...")
                    return None, None, None, None
    except KeyboardInterrupt:
        print("\nStopped by user")
        return None, None, None, None
    finally:
        sock.close()


def main():
    print("OPTITRACK POSITION READER")
    print("=" * 40)
    print("Optitrack coordinate system:")
    print(" +X = left")
    print(" +Y = up")
    print(" +Z = forward")
    print("=" * 40)
    print("Press Ctrl+C to stop\n")

    try:
        while True:
            x, y, z, rb_id = get_optitrack_position()
            if x is not None:
                print(f"Rigid Body ID: {rb_id} X:{x:+7.3f}, Y: {y:+7.3f}, Z: {z:+7.3f}")
                print(f"             LEFT   UP    FWD")
                print(f"Height above ground: {y:+7.3f} m")
                print("-" * 50)
            time.sleep(0.1)  # Adjust the sleep time as needed
    except KeyboardInterrupt:
        print("\nDone AMH19")
if __name__ == "__main__":
    main()
