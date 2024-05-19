"""Envt_state is a global dictionary keeping track of the status 
    of the mobile robots, trays, and parts. It is the sole repository
    where boolean confirmations are made whenever any update is made to
    the actors in this scenario

    Returns:
        null
    Type: dictionary (nested)
    """

envt_state = {'robot_floor': {'hold_part': False,
                              'empty': True},
              'robot_ceiling': {'hold_part': False,
                                'empty': True,
                                'hold_tray': False},
              'table': {'wood_tray_table': True,
                        'metal_tray_table': True},
              'kitting_tray': {'yellow_tray': {'on_table': True,
                                               'on_robot': False,
                                               'on_agv': False},
                               'gray_tray': {'on_table': True,
                                             'on_robot': False,
                                             'on_agv': False}
                               },
              'tray_status': False,
              'red_location': {'bin1': False,
                               'bin2': False,
                               'bin3': False,
                               'bin4': False},
              'green_location': {'bin1': False,
                                 'bin2': False,
                                 'bin3': False,
                                 'bin4': False},
              'blue_location':  {'bin1': False,
                                 'bin2': False,
                                 'bin3': False,
                                 'bin4': False},
              'red_part': 0, 'green_part': 0, 'blue_part': 0,
              'agv': {'agv1': False, 'agv2': False,
                      'agv3': False, 'agv4': False},
              'assembly': {'as1': False, 'as2': False,
                           'as3': False, 'as4': False},
              'parts_In_T': {'red': 0, 'green': 0, 'blue': 0},
              'bins': {'bin1': False, 'bin2': False,
                       'bin3': False, 'bin4': False}}
#   'bins': {'1': 'robot_floor', '2': 'robot_floor',
#            '3': 'robot_ceiling', '4': 'robot_ceiling'}}


def pickup_tray(robot: str, tray: str, table: str):
    """
    `robot` picks up `tray` from `table`

    Note: This function can only be executed if `robot` is 'ceiling_robot'

    Preconditions:
        - `robot` is 'robot_ceiling'
        - `tray` is located on `table`
        - `robot`'s gripper is empty

    Effects:
        - `robot`'s gripper is not empty
        - `robot`'s gripper is holding the tray
        - location of `tray` is not on `table`
        - location of `tray` is in `robot`'s gripper

    Args:
        robot (str): The robot used to perform this action
        tray (str): The tray to pick up
            ('w' - 'yellow_tray or 'm' - 'gray_tray')
        table (str): Table where the tray is located
            ('wood_tray' or 'metal_tray')
    """

    robot_gripper = envt_state[robot]['empty']
    item_tray = ''
    if tray == 'w':
        item_tray = 'wood_tray'
        tray_check = (envt_state['table'][item_tray+'_table'] and
                      envt_state['kitting_tray']
                      ['yellow_tray']['on_table'])

    else:
        item_tray = 'metal_tray'

        tray_check = (envt_state['table'][item_tray+'_table'] and
                      envt_state['kitting_tray']['gray_tray']['on_table'])

    if(robot == 'robot_ceiling' and tray_check and robot_gripper):
        envt_state[robot]['empty'] = False
        envt_state[robot]['hold_tray'] = True

        if tray == 'w':
            envt_state['table']['wood_tray_table'] = False
            envt_state['kitting_tray']['yellow_tray']['on_table'] = False
            envt_state['kitting_tray']['yellow_tray']['on_robot'] = True

        else:
            envt_state['table']['metal_tray_table'] = False
            envt_state['kitting_tray']['gray_tray']['on_table'] = False
            envt_state['kitting_tray']['gray_tray']['on_robot'] = True

        envt_state[robot]['hold_tray'] = True

    print(f"pickup_tray({robot}, {item_tray}, {table})")


def place_tray(robot: str, tray: str, agv: str):
    """
    `robot` places `tray` on `agv`.

    Note: This function can only be executed if `robot` is 'robot_ceiling'.

    Preconditions:
        - `robot` is 'robot_ceiling'
        - `robot`'s gripper is holding `tray`
        - `tray` is located in `robot`'s gripper
        - location of `tray` is in `robot`'s gripper

    Effects:
        - `robot`'s gripper is empty
        - `robot`'s gripper is not holding `tray`
        - location of `tray` is not in `robot`'s gripper
        - location of `tray` is `agv`

    Args:
        robot (str): The robot used to perform this action
        tray (str): The tray to place on `agv`
            ('w' - 'yellow_tray or 'm' - 'gray_tray')
        agv (str): AGV where `tray` is placed
            ('agv1', 'agv2', 'agv3', or 'agv4')
    """

    robot_gripper = (not envt_state[robot]['empty'] and
                     envt_state[robot]['hold_tray'])
    if tray == 'w':
        tray = 'wood_tray'
        tray_check = (not envt_state[robot]['empty'] and
                      envt_state['kitting_tray']
                      ['yellow_tray']['on_robot'])

    elif tray == 'm':
        tray = 'metal_tray'
        tray_check = (not envt_state[robot]['empty'] and
                      envt_state['kitting_tray']['gray_tray']['on_robot'])

    if(robot == 'robot_ceiling' and tray_check and robot_gripper):
        envt_state[robot]['empty'] = True
        envt_state[robot]['hold_tray'] = False
        if tray == 'wood_tray':
            envt_state['kitting_tray']['yellow_tray']['on_robot'] = False
            envt_state['kitting_tray']['yellow_tray']['on_agv'] = True
        else:
            envt_state['kitting_tray']['gray_tray']['on_robot'] = False
            envt_state['kitting_tray']['gray_tray']['on_agv'] = True

    # agv is an integer between 1 - 4

    envt_state['agv'][agv] = True

    print(f"place_tray({robot}, {tray}, {agv})")


def pickup_part(robot: str, bin: str, part: str):
    """
    `robot` picks up `part` from `bin`.

    Preconditions:
        - `robot`'s gripper is empty
        - `bin` has parts of type `part`
        - The number of parts in `bin` is > 0

    Effects:
        - Decrease the number of parts in `bin` by 1

    Args:
        robot (str): Robot used to perform this action
            ('robot_ceiling' or 'robot_floor')
        bin (str): Bin from which a part is removed
            (1 - 'bin1', ..., or 4 - 'bin4')
        part (str): The part to pick up
            ('red_part', 'green_part', or 'blue_part')
    """

    robot_gripper = envt_state[robot]['empty']

    part_no = envt_state[part]

    if (robot_gripper and (bin in '1234') and part_no > 0):

        envt_state[part] -= 1

    print(f"pickup_part({robot}, {bin}, {part})")


def place_part(robot: str, tray: str, part: str, agv: str):
    """
    `robot` places `part` in `tray`, which is located on `agv`

    Preconditions:
        - `robot`'s gripper is holding `part`
        -  location of `tray` is `agv`
        - `tray` is not complete

    Effects:
        - `tray` has `part`
        - `robot`'s gripper is not holding `part`

    Args:
        robot (str): Robot used to perform this action
            ('robot_ceiling' or 'robot_floor')
        tray (str): Tray in which `part` is placed
            ('w' - 'yellow_tray or 'm' - 'gray_tray')
        part (str): The part to place
            ('red_part', 'green_part', or 'blue_part')
        agv (str): AGV where `tray` is located.
            ('agv1', 'agv2', 'agv3', or 'agv4')
    """

    robot_gripper = (not envt_state[robot]['empty'] and
                     envt_state[robot]['hold_part'])
    if tray == 'w':
        tray = 'wood_tray'
        tray_check = (not envt_state[robot]['empty'] and
                      envt_state['kitting_tray']['yellow_tray']['on_agv'])

    elif tray == 'm':
        tray = 'metal_tray'
        tray_check = (not envt_state[robot]['empty'] and
                      envt_state['kitting_tray']['gray_tray']['on_agv'])

    comp = envt_state['tray_status']

    if(robot_gripper and tray_check and comp):

        if part == 'red_part':
            envt_state['parts_In_T']['red'] += 1
        elif part == 'green_part':
            envt_state['parts_In_T']['green'] += 1
        else:
            envt_state['part_in_T']['blue'] += 1

        envt_state[robot]['hold_part'] = False
        envt_state[robot]['empty'] = True

    print(f"place_part({robot}, {tray}, {part}, {agv})")


def ship_agv(agv: str, tray: str, station: str):
    """
    Ship `agv` to assembly station `station` if `tray` is complete

    Preconditions:
        - `tray` is complete (it has all the parts needed)
        - `tray` is on `agv`

    Effects:
        - `agv` location is `station`

    Args:
        agv (str): AGV to ship
            ('agv1', 'agv2', 'agv3', or 'agv4')
        tray (str): Tray located on `agv`
            ('yellow_tray' or 'gray_tray')
        station (str): Assembly station where to ship `agv`
            ('as1', 'as2', 'as3', or 'as4')
    """
    if tray == 'yellow_tray':
        tray_check = envt_state['kitting_tray'][tray]['on_agv']

    elif tray == 'gray_tray':
        tray_check = envt_state['kitting_tray'][tray]['on_agv']

    stat = envt_state['tray_status']

    if stat and tray_check:
        print(f"ship_agv({agv}, {station})")
    else:
        print("NO SOLUTION FOUND - Assembly station not known!")


def initial_state(part):
    """
    Ask user question about desired part.

    Preconditions:
        - no preconditions

    Effects:
        - Record number of parts in the bins

    Args:
        part (str): type of part to be placed on bin
                    (red_part), (green_part), and (blue_part)
    """

    while True:

        parts_in = input(f"How many {part}_parts in the workcell [0, 4, 9]? ")
        parts_in.replace(" ", "")
        if parts_in not in '0123456789':
            print("Non-integer entered. Please Re-enter a number")
            continue
        elif parts_in not in '049':
            print("Wrong number entered. Choose a number from list")
            continue
        envt_state[part+'_part'] = int(parts_in)

        if parts_in != '0':

            used = part
            while True:
                parts_bin = input(
                    f"In which bin are {part}_parts located [1, 2, 3, 4]? ")
                parts_bin.replace(" ", "")
                if parts_bin not in '0123456789':
                    print("Non-integer entered. Please Re-enter a number")
                    continue
                if parts_bin not in '1234':
                    print("Wrong number entered. Choose a number from list")
                    continue
                envt_state[part+'_location']['bin' + parts_bin] = True
                break
        else:
            used = ''
        break
    return used


def goal_state(used):
    """
    Ask user question about desired tray, AGV, assembly station, parts in tray.

    Preconditions:
        - no preconditions

    Effects:
        - Record number of parts in the bins

    Args:
        used (str): type of part introduced by the user in the bins
                    (red_part), (green_part), and (blue_part)
    """
    input_check = True

    while input_check:
        tray_in = input("Which tray to use [(w)ood, (m)etal]? ")
        tray_in.replace(" ", "")
        if tray_in not in "wm":
            print("Incorrect input. Choose either 'w' or 'm'")
            continue
        break

    agv_in = input("Which AGV to use [1, 2, 3, 4]? ")
    agv_in.replace(" ", "")

    while True:
        if agv_in not in "1234":
            print("Incorrect input. Choose a number from given list")
            agv_in = input("Which AGV to use [1, 2, 3, 4]? ")
            agv_in.replace(" ", "")
            continue
        break
    envt_state['agv']['agv' + agv_in] = True

    if agv_in in '12':

        while True:
            ast = input("Which assembly station to ship agv" + agv_in +
                        " [1, 2]? ")
            ast.replace(" ", "")
            if ast not in '12':
                print("Incorrect input. Choose a number from given list")
                ast = input("Which assembly station to ship agv" + agv_in +
                            " [1, 2]? ")
                ast.replace(" ", "")
                continue
            break
        envt_state['assembly']['ast' + ast] = True

    else:

        while True:
            ast = input("Which assembly station to ship agv" + agv_in +
                        " [3, 4]? ")
            ast.replace(" ", "")
            if ast not in '34':
                print("Incorrect input. Choose a number from given list")
                ast = input("Which assembly station to ship agv" + agv_in +
                            " [3, 4]? ")
                ast.replace(" ", "")
                continue
            break
        envt_state['assembly']['ast' + ast] = True

    if 'red' in used:
        r_parts_T = input("How many red_parts in tray [0, 1, 2]? ")
        r_parts_T.replace(" ", "")

        while True:
            if r_parts_T not in '012':
                print("Incorrect input. Choose a number from given list")
                r_parts_T = input("How many red_parts in tray [0, 1, 2]? ")
                r_parts_T.replace(" ", "")
                continue
            else:
                break
        envt_state['parts_In_T']['red'] = int(r_parts_T)

    if 'green' in used:
        g_parts_T = input("How many green_parts in tray [0, 1, 2]? ")
        g_parts_T.replace(" ", "")

        while True:
            if g_parts_T not in '012':
                print("Incorrect input. Choose a number from given list")
                g_parts_T = input("How many green_parts in tray [0, 1, 2]? ")
                g_parts_T.replace(" ", "")
                continue
            else:
                break
        envt_state['parts_In_T']['green'] = int(g_parts_T)

    if 'blue' in used:
        b_parts_T = input("How many blue_parts in tray [0, 1, 2]? ")
        b_parts_T.replace(" ", "")

        while True:
            if b_parts_T not in '012':
                print("Incorrect input. Choose a number from given list")
                b_parts_T = input("How many blue_parts in tray [0, 1, 2]? ")
                b_parts_T.replace(" ", "")
                continue
            else:
                break
        envt_state['parts_In_T']['blue'] = int(b_parts_T)

    goal_inputs = (tray_in, 'agv' + agv_in, 'ast' + ast)
    # Goal -      { tray,    agv robot,     assembly station }

    return goal_inputs


def user_input():
    """
    Obtain information from user about parts, bin, tray, and assembly station

    Preconditions:
        - no preconditions

    Effects:
        - Record number of parts in the bins
        - Record tray, agv, and assembly station type
        - Record parts to be placed in tray

    Args:
        - No args
    """
    print("================================================================")
    print("INITIAL STATE")
    print("================================================================")

    parts = ['red', 'green', 'blue']
    used = []
    for i in parts:
        # Reads part and bin info from user
        used.append(initial_state(i))

    print("================================================================")
    print("GOAL STATE")
    print("================================================================")

    # Read tray, agv, station, and part in tray info
    return goal_state(used)


def generate_plan():
    """
    Create a plan based on user input

    Precondition:
        - user input needs to be collected

    Effects:
        - Design a solution that:
            - picks up and places tray,
            - picks up and place parts (red, green, blue) on tray,
            - picks up and place tray on mobile robot,
            - send mobile robot to assembly station

    Args:
        - No Args
    """
    tray_in, agv, ast = user_input()
    # { tray, agv robot, assembly station}

    if tray_in == 'w':
        table = 'wood_tray_table'
        tray = 'yellow_tray'
    else:
        table = 'metal_tray_table'
        tray = 'gray_tray'

    print("================================================================")
    print("SOLUTION FOUND")
    print("================================================================")

    # First choose selected tray
    pickup_tray('robot_ceiling', tray_in, table)

    # Then place tray on selected mobile robot
    place_tray('robot_ceiling', tray_in, agv)

    # Next pick up selected parts from bin
    red_no = envt_state['parts_In_T']['red']
    if envt_state['red_part'] != 0:
        bins = envt_state['red_location']
        bins_val = list(bins.values())
        bin_red = bins_val.index(True)+1
        if bin_red == 1 or bin_red == 2:
            robot = 'robot_floor'
        else:
            robot = 'robot_ceiling'
        for i in range(red_no):
            pickup_part(robot, 'bin' + str(bin_red), 'red_part')
            place_part(robot, tray_in, 'red_part', agv)

    green_no = envt_state['parts_In_T']['green']
    if envt_state['green_part'] != 0:
        bins = envt_state['green_location']
        bins_val = list(bins.values())
        bin_green = bins_val.index(True)+1
        if bin_green == 1 or bin_green == 2:
            robot = 'robot_floor'
        else:
            robot = 'robot_ceiling'
        for i in range(green_no):
            pickup_part(robot, 'bin' + str(bin_green), 'green_part')
            place_part(robot, tray_in, 'green_part', agv)

    blue_no = envt_state['parts_In_T']['blue']
    if envt_state['blue_part'] != 0:
        bins = envt_state['blue_location']
        bins_val = list(bins.values())
        bin_blue = bins_val.index(True)+1
        if bin_blue == 1 or bin_blue == 2:
            robot = 'robot_floor'
        else:
            robot = 'robot_ceiling'
        for i in range(blue_no):
            pickup_part(robot, 'bin' + str(bin_blue), 'blue_part')
            place_part(robot, tray_in, 'blue_part', agv)

    envt_state['tray_status'] = True

    # Ship mobile robot to assembly station
    ship_agv(agv, tray, ast)

############# Code ends here #####################
