# Socket params
PORT_NUM = 9000

# Home position
HOME = posj(-90, 0, -90, 0, -90, 0)

# Task 2 Params
MEAS_UP = [
    [200, 500, 100, 0, -180, 0],
    [0, 500, 100, 0, -180, 0],
    [-200, 500, 100, 0, -180, 0]
]
PLACE_UP = [
    [200,  600, 100, 0, -180, 0],
    [0, 600, 100, 0, -180, 0],
    [-200, 600, 100, 0, -180, 0]
]

Z_LIMIT = 3.0
PLACE_MARGIN = 2.0
MIN_DEPTH = 7.5

# Force Params
STX = [3000, 3000, 10, 200, 200, 200]
Z_FORCE = 20.0
CONTACT_FORCE = 5.0

# Speed Params
VJ, AJ = 30, 60
VL, AL = 100, 200
VL_DOWN = 50

# Connection
sock = None
rx_buf = ""

def wait_client():
    global sock
    sock = server_socket_open(PORT_NUM)
    tp_log("[SOCK] PC client connected")

def send_msg(msg):
    res = server_socket_write(sock, (msg + "\n").encode())
    if res < 0:
        tp_log("[SOCK] send failed({0}) : {1}".format(res, msg))

    return res

def send_pos(tcp = None):
    """현재 TCP X/Y/Z와 J1~J6을 GUI의 POS 프로토콜로 전송한다."""
    if tcp is None:
        tcp, _ = get_current_posx(DR_BASE)
    joint = get_current_posj()
    send_msg("POS,{0:.3f},{1:.3f},{2:.3f},{3:.3f},{4:.3f},{5:.3f},{6:.3f},{7:.3f},{8:.3f}".format(
        tcp[0], tcp[1], tcp[2],
        joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]))

def read_line():
    global rx_buf
    while True:
        idx = rx_buf.find("\n")
        if idx >= 0:
            line = rx_buf[:idx]
            rx_buf = rx_buf[idx + 1:]
            return line

        res, data = server_socket_read(sock)
        if res < 0:
            tp_log("[SOCK] read failed({0})".format(res))
            return None

        rx_buf += data.decode()


# Robot Control Function
def gripper_close():
    set_tool_digital_output(-1)
    set_tool_digital_output(-2)
    wait(0.5)
    set_tool_digital_output(1)
    wait(0.5)

def gripper_open():
    set_tool_digital_output(-1)
    set_tool_digital_output(-2)
    wait(0.5)
    set_tool_digital_output(2)
    wait(0.5)
    
def move_jx(pos, v = VJ, a = AJ):
    send_msg("MoveJ Start")
    movejx(pos, v, a, sol=SOL)
    send_msg("MoveJ Done")
    send_pos()

def move_j(pos, v = VJ, a = AJ):
    send_msg("MoveJ Start")
    movej(pos, v, a)
    send_msg("MoveJ Done")
    send_pos()

def move_l(pos, v = VL, a = AL):
    send_msg("MoveL Start")
    movel(pos, v, a)
    send_msg("MoveL Done")
    send_pos()

def move_l_REL(dist, v = VL, a = AL):
    send_msg("MoveL_REL Start")
    movel(dist, mod=DR_MV_MOD_REL, v=v, a=a)
    send_msg("MoveL_REL Done")
    send_pos()

def grip_on():
    send_msg("Gripper ON")
    gripper_close()

def grip_off():
    send_msg("Gripper OFF")
    gripper_open()

def with_z(p, z):
    return posx(p[0], p[1], z, p[3], p[4], p[5])


# Task 1 : Pick and Place
T1_HOME_1   = posx(0, 500, 100, 0, 180, 0) 
T1_HOME_2   = posx(0, 700, 100, 0, 180, 0)

T1_PICK_UP_1  = posj(-111.14, -30.82, -86.17, 0, -63.02, -111.14)
T1_PLACE_UP_1 = posj(-67.53, -30.82, -86.17, 0, -63.02, -67.53)

T1_PICK_UP_2  = posj(-105.45, -61.2, -30.66, 0, -88.14, -105.45)
T1_PLACE_UP_2 = posj(-73.56, -61.2, -30.66, 0, -88.14, -73.56)

Z_UP = posx(0, 0, 90, 0, 0, 0)
Z_UP_PLACE = posx(0, 0, 95, 0, 0, 0)
Z_DN = posx(0, 0, -90, 0, 0, 0)
Z_DN_PLACE = posx(0, 0, -85, 0, 0, 0)

VEL, ACC = 60, 60

def task1_pick_place(home, pick, place):
    send_msg("Return HOME")
    move_l(home, 100, 80)

    move_j(pick, VEL, ACC)
    move_l_REL(Z_DN, VEL, ACC)

    grip_on()

    move_l_REL(Z_UP, VEL, ACC)

    move_j(place, VEL, ACC)
    move_l_REL(Z_DN_PLACE, VEL, ACC)

    grip_off()

    move_l_REL(Z_UP_PLACE, VEL, ACC)

    move_l(home, VEL, ACC)

def TASK1():
    gripper_open()

    task1_pick_place(T1_HOME_1, T1_PICK_UP_1, T1_PLACE_UP_1)
    task1_pick_place(T1_HOME_2, T1_PICK_UP_2, T1_PLACE_UP_2)


# Task 2: Height Measurement - Alignment - Pick and Place
def height_measurement(job_id, p_up):
    move_jx(p_up)

    send_msg("Compliance start,{0}".format(job_id))
    task_compliance_ctrl(STX)
    set_desired_force(fd=[0, 0, -Z_FORCE, 0, 0, 0],
                      dir = [0,0,1,0,0,0],
                      mod=DR_FC_MOD_REL)
    wait(0.2)

    contact = False

    while True:
        if check_force_condition(DR_AXIS_Z, min = CONTACT_FORCE, max = CONTACT_FORCE + 5, ref = DR_BASE):
            contact = True
            break
        cur, _ = get_current_posx(DR_BASE)
        if cur[2] < Z_LIMIT:
            break
        wait(0.01)

    cur, _ = get_current_posx(DR_BASE)
    contact_z = cur[2]
    send_pos(cur)

    release_force()
    release_compliance_ctrl()
    send_msg("Compliance finish,{0}".format(job_id))

    move_l(p_up)

    if not contact:
        send_msg("ERROR,{0},no contact".format(job_id))
        return None
    if p_up[2] - contact_z < 1.0:     # 출발하자마자 걸림 = 센서 오프셋/툴 무게 설정 문제
        send_msg("ERROR,{0},false trigger".format(job_id))
        return None
    return contact_z

def TASK2():
    grip_off()

    results = []

    for i in range(len(MEAS_UP)):
        job_id = i + 1

        h = height_measurement(job_id, MEAS_UP[i])
        if h is None:
            return False
        p = MEAS_UP[i]
        send_msg("DATA,{0},{1:.2f},{2:.2f},{3:.2f},{4:.2f}".format(
            job_id, p[0], p[1], p[2], h))
        results.append([job_id, h])

    order = sorted(results, key=lambda r: r[1], reverse = True)
    send_msg("ORDER," + ",".join([str(r[0]) for r in order]))

    for slot in range(len(order)):
        job_id = order[slot][0]
        h = order[slot][1]
        p_up = MEAS_UP[job_id - 1]
        grip_z = max(h, MIN_DEPTH)
        place_z = grip_z + PLACE_MARGIN

        grip_off()
        move_jx(p_up)
        move_l(with_z(p_up, grip_z), v=VL_DOWN)
        grip_on()
        move_l(p_up)
        move_jx(PLACE_UP[slot])
        move_l(with_z(PLACE_UP[slot], place_z), v=VL_DOWN)
        grip_off()
        move_jx(PLACE_UP[slot])

    movej(HOME, v=VJ, a=AJ)

    return True


# Main: server open - wait for order - action
movej(HOME, v=VJ, a=AJ)
gripper_open()
SOL = get_current_solution_space()

wait_client()
send_msg("READY")
send_pos()

while True:
    cmd = read_line()

    if cmd is None:
        tp_log("[SOCK] client disconnected")
        break

    if cmd == "TASK,1":
        send_msg("TASK 1 Accepted")
        TASK1()
        send_msg("DONE,1")

    elif cmd == "TASK,2":
        send_msg("TASK 2 Accepted")
        ok = TASK2()
        if ok:
            send_msg("DONE,2")
        else:
            send_msg("Task 2 Failed")

    elif cmd == "END":
        send_msg("END Received")
        break

    else:
        send_msg("UNKNOWN " + cmd)

server_socket_close(sock)
tp_log("[MAIN] program end")
