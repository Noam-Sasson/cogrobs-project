from unified_planning.shortcuts import *
from unified_planning.model import Problem, Fluent, Action, Object
from unified_planning.engines import engine
from unified_planning.engines.sequential_simulator import UPSequentialSimulator
from unified_planning.model.metrics import MaximizeExpressionOnFinalState
from unified_planning.plans.plan import PlanKind
from unified_planning.engines import OptimalityGuarantee, PlanGenerationResultStatus
from collections import defaultdict
from itertools import product
import time
from unified_planning.plans.time_triggered_plan import _get_timepoint_effects, _extract_action_timings, _get_timepoint_simulated_effects

def solve_problem(initial_state_dict : dict):
    '''
     initial_state_dict should countain the following:
    - server_1: {position, holds}
    - cleaer: {position, cleaning}
    - host: {position, seating_customers}
    - orders: {table_name: {food_order, food_prep_time, can_be_taken, in_making, eating_time, cleaning_time, pondering_time, ready_to_order, order_taken , served, time_bais}}
    - customers: {customer_id: {following, table, seated, eaten, party_size}}
    - dishes: {dish: {used, used_by}}
    - revenue: current revenue
    '''
    # Create the planning problem
    
    if initial_state_dict is None:
        CUST_COUNT = 5
    else:
        CUST_COUNT = len(initial_state_dict['customers'])
    
    if initial_state_dict is None:
        DISH_COUNT = 2
    else:
        DISH_COUNT = len(initial_state_dict['dishes'])
    
    PADDING = 1

    problem = Problem("DinerProblem")

    # Define types of objects (Table, Robot, Customer)
    Table = UserType("Table")
    Host = UserType("Host")
    Cleaner = UserType("Cleaner")
    Server = UserType("Server")
    Customer = UserType("Customer")
    Location = UserType("Location")
    # Order = UserType('Order')
    Dish = UserType('Dish')
    # Define objects in the problem

    if initial_state_dict is None:
        tables = [
            table_tl := Object('table_tl', Table),
            table_bl := Object('table_bl', Table),
            table_tr := Object('table_tr', Table),
            table_br := Object('table_br', Table),
        ]
    else:
        tables = [Object(k, Table) for k in initial_state_dict['orders'].keys()]
    # fake_table = Object('fake_table', Table)
    locations = [
        (kitchen := Object("k_in", Location)),
        (TL1 := Object('tl_1', Location)),
        (TL2 := Object('tl_2', Location)),
        (ML1 := Object('ml_1', Location)),
        (ML2 := Object('ml_2', Location)),
        (BL1 := Object('bl_1', Location)),
        (BL2 := Object('bl_2', Location)),
        (TR1 := Object('tr_1', Location)),
        (TR2 := Object('tr_2', Location)),
        (MR1 := Object('mr_1', Location)),
        (MR2 := Object('mr_2', Location)),
        (BR1 := Object('br_1', Location)),
        (BR2 := Object('br_2', Location)),
        (ML := Object('ml', Location)),
        (MR := Object('mr', Location)),
        (TBL_TL := Object('tbl_tl', Location)),
        (TBL_BL := Object('tbl_bl', Location)),
        (TBL_TR := Object('tbl_tr', Location)),
        (TBL_BR := Object('tbl_br', Location)),
        (K := Object('k', Location)),
        # (fake_location := Object('fake_location', Location))
    ]
    robots = [
        host := Object("host", Host),
        server_1 := Object("server_1", Server),
        # server_2 := Object("server_2", Robot),
        cleaner := Object("cleaner", Cleaner)

    ]
    if initial_state_dict is None:
        customers = [Object(f"customer_{i}", Customer) for i in range(1, CUST_COUNT+1)]
    else:
        customers = [Object(k , Customer) for k in initial_state_dict['customers'].keys()]
    # fake_customer = Object('fake_customer', Customer)
    # orders = [Object(f'order_{table}', Order) for table in tables]
    # fake_order = Object('fake_order', Order)
    if initial_state_dict is None:
        dishes = [Object(f'dish_{i}', Dish) for i in range(1, DISH_COUNT+1)]
    else:
        dishes = [Object(k, Dish) for k in initial_state_dict['dishes'].keys()]

    problem.add_objects(locations + tables + robots + customers + dishes)
    # Predicates
    Adjacent = Fluent("Adjacent", BoolType(), loc1=Location,
                      loc2=Location)  # location1 is adjacent to location2
    Distance = Fluent("Distance", IntType(), loc1=Location, loc2=Location)
    Distance_To_Home = Fluent("Distance_To_Home", IntType(), loc=Location)
    TL1_Distances ={
        TL1: 0,
        TL2: 17,
        ML: 12,
        ML1: 12,
        ML2: 13,
        BL1: 17,
        BL2: 17,
        TBL_TL: 9,
        TBL_BL: 13,
        TR1: 19,
        TR2: 31,
        MR: 22,
        MR1: 22,
        MR2: 27,
        BR1: 19,
        BR2: 30,
        TBL_TR: 25,
        TBL_BR: 25,
        K: 25,
        kitchen: 26
    }

    TL1_Distances = {k: v + PADDING for k, v in TL1_Distances.items()}
    '''
    actual drone distances
    tl_2: 18.560000000000002
    ml: 11.520000000000003
    ml_1: 11.584000000000003
    ml_2: 13.376000000000005
    bl_1: 16.895999999999987
    bl_2: 16.960000000000008
    tbl_tl: 9.280000000000001
    tbl_bl: 13.120000000000005
    tr_1: 19.456000000000017
    tr_2: 30.976
    mr: 24.767999999999972
    mr_1: 21.56800000000004
    mr_2: 26.879999999999995
    br_1: 19.392000000000053
    br_2: 30.464000000000055
    tbl_tr: 24.639999999999986
    tbl_br: 24.767999999999915
    k: 24.831999999999994
    k_in: 26.24000000000001
    '''
    Distances = {
        (kitchen, K): 6,
        (K, TR1): 14,
        (K, TR2): 13,
        (TL2, TL1): 22,
        (TL1, ML1): 12,
        (ML1, BL1): 12,
        (BL1, BL2): 22,
        (BL2, ML2): 13,
        (ML2, TL2): 12,
        (ML, ML1): 7,
        (ML, ML2): 8,
        (ML, TBL_TL): 7,
        (ML, TBL_BL): 6,
        (TR1, MR1): 13,
        (MR1, BR1): 13,
        (BR1, BR2): 22,
        (BR2, MR2): 12,
        (MR2, TR2): 13,
        (MR, MR1): 7,
        (MR, MR2): 7,
        (MR, TBL_TR): 7,
        (MR, TBL_BR): 7,
        (ML2, MR1): 13,
        (BL2, BR1): 9,
    }

    Distances = {k: v + PADDING for k, v in Distances.items()}
    '''
    actual air distances
    (kin, k): 6.432
    (tr1, k): 14.048
    (mr1, tr1): 12.96
    (mr1, br1): 12.928000000000004
    (br1, br2): 21.823999999999998
    (mr2, br2): 11.807999999999993
    (mr2, mr): 7.199999999999989
    (tbl_tr, mr): 7.007999999999981
    (mr, tbl_br): 6.463999999999999
    (mr1, mr): 6.8799999999999955
    (tr2, mr2): 12.73599999999999
    (tr2, k): 12.960000000000008
    (br1, bl2): 9.343999999999994
    (bl1, bl2): 21.47200000000001
    (bl1, ml1): 11.935999999999979
    (tl1, ml1): 12.127999999999986
    (tl1, tl2): 22.144000000000005
    (ml2, tl2): 12.127999999999986
    (ml2, bl2): 12.543999999999983
    (ml2, ml): 7.584000000000003
    (tbl_tl, ml): 6.7520000000000095
    (tbl_bl, ml): 6.271999999999991
    (ml1, ml): 7.039999999999964
    (mr1, ml2): 13.343999999999994
    '''

    '''
    actual air distances
    (tl2, tl1): 21.504
    (ml2, tl2): 11.424000000000003
    (ml2, bl2): 11.647999999999996
    (bl2, bl1): 18.656000000000006
    (ml1, bl1): 11.423999999999992
    (ml1, tl1): 11.616
    (ml1, ml): 7.743999999999986
    (ml2, ml): 7.744
    (tbl_tl, ml): 7.7120000000000175
    (tbl_bl, ml): 7.711999999999989
    (bl2, br1): 9.151999999999987
    (mr1, br1): 11.424000000000007
    (tr1, mr1): 11.647999999999996
    (tr1, k): 10.304000000000002
    (tr2, k): 9.120000000000005
    (mr2, tr2): 11.391999999999996
    (mr2, br2): 10.496000000000038
    (br1, br2): 19.423999999999978
    (mr, mr2): 7.711999999999989
    (mr, tbl_tr): 7.711999999999989
    (mr, tbl_br): 7.744000000000028
    (mr, mr1): 7.7760000000000105
    (k, kin): 7.520000000000039
    (ml2, mr1): 14.271999999999991
    '''
    problem.add_fluent(Adjacent, default_initial_value=False)
    problem.add_fluent(Distance_To_Home, default_initial_value=0)
    
    total_distances = defaultdict(int, Distances | {(loc2, loc1): dist for (loc1, loc2), dist in Distances.items()})
    for loc1, loc2 in product(locations, repeat=2):
        problem.set_initial_value(Distance(loc1, loc2), total_distances[(loc1, loc2)])
        problem.set_initial_value(Distance(loc2, loc2), total_distances[(loc2, loc1)])
        if total_distances[(loc1, loc2)]:
            problem.set_initial_value(Adjacent(loc1, loc2), True)
            problem.set_initial_value(Adjacent(loc2, loc1), True)
        else:
            problem.set_initial_value(Adjacent(loc1, loc2), False)
            problem.set_initial_value(Adjacent(loc2, loc1), False)
        
    for loc in locations:
        problem.set_initial_value(Distance_To_Home(loc), TL1_Distances[loc])

    # Job = Fluent('Job', IntType(), robot=Robot)
    # IsAerial = Fluent('IsAerial', BoolType(), robot=Robot)
    # HOST, SERVER, CLEANER = 0, 1, 2
    # Fluents
    Host_At = Fluent('Host_At', BoolType(), host=Host, loc=Location)
    Server_At = Fluent('Server_At', BoolType(), server=Server, loc=Location)
    Cleaner_At = Fluent('Cleaner_At', BoolType(), cleaner=Cleaner, loc=Location)
    Table_At = Fluent('Table_At', BoolType(), table=Table, loc=Location)

    Occupied = Fluent("Occupied", BoolType(), table=Table)
    Clean = Fluent("Clean", BoolType(), table=Table)

    Seating_Customers = Fluent('Seating_Customer', BoolType(), robot=Host)
    Following = Fluent('Following', BoolType(), robot=Host, customer=Customer)

    In_Rest = Fluent('In_Rest', BoolType(), customer=Customer)
    Seated = Fluent('Seated', BoolType(), customer=Customer)
    Ready_To_Order = Fluent('Ready_To_Order', BoolType(), table=Table)
    Order_Taken = Fluent('Order_Taken', BoolType(), table=Table)
    Served = Fluent("Served", BoolType(), table=Table)
    Eaten = Fluent("Eaten", BoolType(), customer=Customer)
    Seated_At = Fluent('Seated_At', BoolType(), customer=Customer, table=Table)
    Party_Size = Fluent('Party_Size', IntType(), customer=Customer)
    Used = Fluent('Used', BoolType(), dish = Dish)
    Order_In_Making = Fluent('In_Making', BoolType(), table = Table)
    Used_by = Fluent('Used_by', BoolType(), dish = Dish, table = Table)
    Cleaning = Fluent('Cleaning', BoolType(), cleaner = Cleaner)

    

    if initial_state_dict is None:
        for customer in customers:
            problem.set_initial_value(Party_Size(customer), 2)

    Holds = Fluent('Holds', BoolType(), server = Server)
    Serves = Fluent('serves', BoolType(), server = Server, table =Table)

    # Owner = Fluent('Owner', Customer, order=Order)
    Food_Order = Fluent('Food', IntType(), table = Table)
    Food_Prep_Time = Fluent('Food_Prep_Time', IntType(), table = Table)
    Cleaning_Time = Fluent('Cleaning_Time', IntType(), table = Table)
    Ordering_Time = Fluent('Ordering_Time', IntType(), table = Table)
    Eating_Time = Fluent('Eating_Time', IntType(), table = Table)
    Can_Be_Taken = Fluent('Can_Be_Taken', BoolType(), table = Table)

    Stood_In = Fluent('Stood_In', BoolType(), loc=Location)

    Revenue = Fluent('Revenue', IntType())
    problem.add_fluent(Revenue, default_initial_value=0)

    # Define actions
    # pick up customers
    pick_up_customers = InstantaneousAction('take_customers', rob=Host, cust=Customer)
    rob = pick_up_customers.parameter('rob')
    cust = pick_up_customers.parameter('cust')

    pick_up_customers.add_precondition(Not(In_Rest(cust)))
    pick_up_customers.add_precondition(Host_At(rob, TL1))
    pick_up_customers.add_precondition(Not(Seating_Customers(rob)))


    pick_up_customers.add_effect(Seating_Customers(rob), True)
    pick_up_customers.add_effect(Following(rob, cust), True)
    pick_up_customers.add_effect(In_Rest(cust), True)

    # seat customers
    seat_customers = InstantaneousAction('seat_customers', rob=Host, table=Table, cust=Customer, loc = Location)
    rob = seat_customers.parameter('rob')
    table = seat_customers.parameter('table')
    cust = seat_customers.parameter('cust')
    loc = seat_customers.parameter('loc')

    seat_customers.add_precondition(Seating_Customers(rob))
    seat_customers.add_precondition(Following(rob, cust))
    seat_customers.add_precondition(Host_At(rob, loc))
    seat_customers.add_precondition(Table_At(table, loc))
    seat_customers.add_precondition(Not(Occupied(table)))
    seat_customers.add_precondition(Clean(table))

    seat_customers.add_effect(Seating_Customers(rob), False)
    seat_customers.add_effect(Following(rob, cust), False)

    seat_customers.add_effect(Occupied(table), True)
    seat_customers.add_effect(Seated_At(cust, table), True)
    seat_customers.add_effect(Seated(cust), True)

    # clean a table
    clean_table = DurativeAction('clean_table', rob=Cleaner, table=Table, loc = Location)
    rob = clean_table.parameter('rob')
    table = clean_table.parameter('table')
    loc = clean_table.parameter('loc')

    clean_table.set_fixed_duration(CLEANING_TIME := Cleaning_Time(table))  # TODO: Decide on Cleaning time
    clean_table.add_condition(StartTiming(),Cleaner_At(rob, loc))
    clean_table.add_condition(StartTiming(),Table_At(table, loc))  
    clean_table.add_condition(StartTiming(), Not(Occupied(table)))
    clean_table.add_condition(StartTiming(), Not(Clean(table)))
    clean_table.add_condition(StartTiming(), Not(Cleaning(rob)))
    clean_table.add_effect(StartTiming(), Cleaning(rob), True)

    clean_table.add_effect(EndTiming(), Clean(table), True)
    clean_table.add_effect(EndTiming(), Cleaning(rob), False)

    # decide order
    decide_order = DurativeAction('decide_order', table=Table, cust=Customer)
    cust = decide_order.parameter('cust')
    table = decide_order.parameter('table')

    decide_order.set_fixed_duration(ORDERING_TIME := Ordering_Time(table))  # TODO: Decide on Ordering time
    decide_order.add_condition(StartTiming(), Seated(cust))
    decide_order.add_condition(StartTiming(), Not(Ready_To_Order(table)))
    decide_order.add_condition(StartTiming(), Not(Order_Taken(table)))
    decide_order.add_condition(StartTiming(), Not(Served(table)))
    decide_order.add_condition(StartTiming(), Seated_At(cust, table))

    decide_order.add_effect(EndTiming(), Ready_To_Order(table), True)

    # actually order
    take_order = InstantaneousAction(f'take_order', rob=Server, table=Table, dish=Dish, loc = Location)
    rob = take_order.parameter('rob')
    # cust = take_order.parameter('cust')
    table = take_order.parameter('table')
    # order = take_order.parameter('order')
    dish = take_order.parameter('dish')
    loc = take_order.parameter('loc')

    take_order.add_precondition(Not(Used(dish)))
    take_order.add_precondition(Ready_To_Order(table))
    # take_order.add_precondition(Equals(Seated_At(cust), table))
    take_order.add_precondition(Server_At(rob, loc))
    take_order.add_precondition(Table_At(table, loc))
    take_order.add_precondition(Not(Order_Taken(table)))
    take_order.add_precondition(Not(Served(table)))
    # take_order.add_precondition(Equals(Owner(order), table))

    take_order.add_effect(Used_by(dish, table), True)
    take_order.add_effect(Used(dish), True)
    take_order.add_effect(Order_Taken(table), True)
    # take_order.add_effect(Food_Order(table), 20)#Times(Party_Size(cust), 20)) to be handled by simulation

    # prepare the food
    make_food = DurativeAction(f'make_order', table = Table, dish=Dish)
    
    # order = make_food.parameter('order')
    table = make_food.parameter('table')
    make_food.set_fixed_duration(total_time := Food_Prep_Time(table))#Times(Party_Size(Owner(order)), PREP_TIME))

    make_food.add_condition(StartTiming(), Used_by(dish, table))
    # make_food.add_condition(StartTiming(), GT(Food_Order(table), 0)) # order has food
    make_food.add_condition(StartTiming(), Not(Order_In_Making(table)))
    make_food.add_condition(StartTiming(), Not(Can_Be_Taken(table)))

    make_food.add_effect(StartTiming(), Order_In_Making(table), True)

    make_food.add_effect(EndTiming(), Can_Be_Taken(table), True)
    make_food.add_effect(EndTiming(), Order_In_Making(table), False)

    # take food
    take_food = InstantaneousAction('take_food', rob=Server, table = Table, dish=Dish)
    rob = take_food.parameter('rob')
    # order = take_food.parameter('order')
    table = take_food.parameter('table')

    take_food.add_precondition(Used_by(dish, table))
    take_food.add_precondition(Not(Holds(rob)))
    take_food.add_precondition(Server_At(rob, kitchen))
    take_food.add_precondition(Can_Be_Taken(table))

    take_food.add_effect(Holds(rob), True)
    take_food.add_effect(Serves(rob, table), True)
    take_food.add_effect(Can_Be_Taken(table), False)
    take_food.add_effect(Used(dish), False)
    take_food.add_effect(Used_by(dish, table), False)

    # give food
    serve_food = InstantaneousAction('serve_food', rob=Server, table = Table, loc = Location)
    rob = serve_food.parameter('rob')
    # cust = serve_food.parameter('cust')
    table = serve_food.parameter('table')
    loc = serve_food.parameter('loc')

    serve_food.add_precondition(Holds(rob))
    serve_food.add_precondition(Serves(rob, table))
    serve_food.add_precondition(Server_At(rob, loc))
    serve_food.add_precondition(Table_At(table, loc))

    serve_food.add_effect(Holds(rob), False)
    serve_food.add_effect(Served(table), True)
    serve_food.add_effect(Serves(rob, table), False)

    # eat food
    eat = DurativeAction('eat', cust=Customer, table=Table)
    cust = eat.parameter('cust')
    table = eat.parameter('table')

    eat.set_fixed_duration(EATING_TIME := Eating_Time(table))  # TODO: Decide on Eating time
    eat.add_condition(StartTiming(), Served(table))
    eat.add_condition(StartTiming(), Seated_At(cust, table))

    eat.add_effect(EndTiming(), Occupied(table), False)
    eat.add_effect(EndTiming(), Clean(table), False)
    eat.add_effect(EndTiming(), Eaten(cust), True)
    eat.add_increase_effect(EndTiming(), Revenue, 20) #Times(Party_Size(cust), 20)) ## will be insreted by simulation
    
    eat.add_effect(EndTiming(), Served(table), False)
    eat.add_effect(EndTiming(), Ready_To_Order(table), False)
    eat.add_effect(EndTiming(), Order_Taken(table), False)
    # eat.add_effect(EndTiming(), Food_Order(table), 0)
    eat.add_effect(EndTiming(), Seated_At(cust, table), False)

    host_move = DurativeAction('host_move', rob=Host, loc1=Location, loc2=Location)
    rob = host_move.parameter('rob')
    loc1 = host_move.parameter('loc1')
    loc2 = host_move.parameter('loc2')
    host_move.set_fixed_duration(Distance(loc1, loc2))

    host_move.add_condition(StartTiming(), Adjacent(loc1, loc2))
    host_move.add_condition(StartTiming(), Host_At(rob, loc1))
    host_move.add_condition(StartTiming(), Seating_Customers(rob))
    host_move.add_condition(EndTiming(), Not(Stood_In(loc2)))

    host_move.add_effect(StartTiming(), Stood_In(loc1), False)
    host_move.add_effect(EndTiming(), Host_At(rob, loc2), True)
    host_move.add_effect(StartTiming(), Host_At(rob, loc1), False)
    host_move.add_effect(EndTiming(), Stood_In(loc2), True)

    server_move = DurativeAction('server_move', rob=Server, loc1=Location, loc2=Location)
    rob = server_move.parameter('rob')
    loc1 = server_move.parameter('loc1')
    loc2 = server_move.parameter('loc2')
    server_move.set_fixed_duration(Distance(loc1, loc2))

    server_move.add_condition(StartTiming(), Adjacent(loc1, loc2))
    server_move.add_condition(StartTiming(), Server_At(rob, loc1))
    server_move.add_condition(EndTiming(), Not(Stood_In(loc2)))

    server_move.add_effect(StartTiming(), Stood_In(loc1), False)
    server_move.add_effect(EndTiming(), Server_At(rob, loc2), True)
    server_move.add_effect(StartTiming(), Server_At(rob, loc1), False)
    server_move.add_effect(EndTiming(), Stood_In(loc2), True)

    cleaner_move = DurativeAction('cleaner_move', rob=Cleaner, loc1=Location, loc2=Location)
    rob = cleaner_move.parameter('rob')
    loc1 = cleaner_move.parameter('loc1')
    loc2 = cleaner_move.parameter('loc2')
    cleaner_move.set_fixed_duration(Distance(loc1, loc2))

    cleaner_move.add_condition(StartTiming(), Adjacent(loc1, loc2))
    cleaner_move.add_condition(StartTiming(), Cleaner_At(rob, loc1))
    cleaner_move.add_condition(StartTiming(), Not(Stood_In(loc2)))
    cleaner_move.add_condition(EndTiming(), Not(Cleaning(rob)))

    cleaner_move.add_effect(StartTiming(), Stood_In(loc1), False)
    cleaner_move.add_effect(EndTiming(), Cleaner_At(rob, loc2), True)
    cleaner_move.add_effect(StartTiming(), Cleaner_At(rob, loc1), False)
    cleaner_move.add_effect(EndTiming(), Stood_In(loc2), True)

    go_home = DurativeAction('go_home', rob=Host, loc1=Location)
    rob = go_home.parameter('rob')
    loc1 = go_home.parameter('loc1')
    go_home.set_fixed_duration(Distance_To_Home(loc1)) # TODO: Decide on going home time

    go_home.add_condition(EndTiming(), Not(Stood_In(TL1)))
    go_home.add_condition(StartTiming(), Host_At(rob, loc1))
    # go_home.add_condition(StartTiming(), Not(Host_At(rob, TL1)))
    go_home.add_condition(StartTiming(), Not(Seating_Customers(rob)))
    # go_home.add_condition(StartTiming(), Not(Stood_In(loc1)))

    go_home.add_effect(StartTiming(), Stood_In(loc1), False)
    # go_home.add_effect(StartTiming(), Stood_In(TL1), False)
    go_home.add_effect(EndTiming(), Host_At(rob, TL1), True)
    go_home.add_effect(StartTiming(), Host_At(rob, loc1), False)
    go_home.add_effect(EndTiming(), Stood_In(TL1), True)

    # Initial state
    # problem.add_fluents([Host_At, Server_At, Cleaner_At, Table_At, Adjacent,  Seated_At])
    # for order in orders+[fake_order]:
    #     problem.set_initial_value(Owner(order), fake_customer)
    
    
    problem.add_fluent(Serves, default_initial_value=False)
    problem.add_fluent(Host_At, default_initial_value=False)
    problem.add_fluent(Server_At, default_initial_value=False)
    problem.add_fluent(Cleaner_At, default_initial_value=False)
    problem.add_fluent(Table_At, default_initial_value=False)
    # problem.add_fluent(Adjacent, default_initial_value=False)
    problem.add_fluent(Seated_At, default_initial_value=False)

    if initial_state_dict is None:
        for dish in dishes:
            problem.set_initial_value(Used(dish), False)

    # for customer in customers+[fake_customer]:
    #     problem.set_initial_value(Seated_At(customer), fake_table)

    problem.add_fluent(Cleaning, default_initial_value=False)
    # problem.set_initial_value(Occupied(fake_table), True)
    problem.add_fluent(Stood_In, default_initial_value=False)

    

    if initial_state_dict is None:
        problem.set_initial_value(Host_At(host, TL1), True)  # Host starts at the entrance
        problem.set_initial_value(Stood_In(TL1), True)
        problem.set_initial_value(Server_At(server_1, K), True)  # Server starts at k
        # problem.set_initial_value(At(server_2), BL2)  # Server starts at k
        # problem.set_initial_value(Stood_In(BL2), True)
        problem.set_initial_value(Stood_In(K), True)
        problem.set_initial_value(Cleaner_At(cleaner, kitchen), True)  # Cleaner starts at the kitchen
        problem.set_initial_value(Stood_In(kitchen), True)
        
        problem.set_initial_value(Table_At(table_bl, TBL_BL), True)
        problem.set_initial_value(Table_At(table_tl, TBL_TL), True)
        problem.set_initial_value(Table_At(table_tr, TBL_TR), True)
        problem.set_initial_value(Table_At(table_br, TBL_BR), True)
        
    problem.add_fluent(Cleaning_Time, default_initial_value = 60)
    problem.add_fluent(Food_Prep_Time, default_initial_value = 60)
    problem.add_fluent(Ordering_Time, default_initial_value = 60)
    problem.add_fluent(Eating_Time, default_initial_value = 60)
    # problem.set_initial_value(Job(server_2), SERVER)
    # problem.set_initial_value(Table_At(fake_table), fake_location)

    problem.add_fluent(Occupied, default_initial_value=False)
    problem.add_fluent(Clean, default_initial_value=True)

    problem.add_fluent(Seating_Customers, default_initial_value=False)
    problem.add_fluent(Following, default_initial_value=False)

    problem.add_fluent(Seated, default_initial_value=False)
    problem.add_fluent(Ready_To_Order, default_initial_value=False)
    problem.add_fluent(Order_Taken, default_initial_value=False)
    problem.add_fluent(Served, default_initial_value=False)
    problem.add_fluent(Party_Size, default_initial_value=0)
    problem.add_fluent(Eaten, default_initial_value=False)

    problem.add_fluent(Used, default_initial_value=False)
    problem.add_fluent(Can_Be_Taken, default_initial_value=False)
    problem.add_fluent(Food_Order, default_initial_value=0)
    problem.add_fluent(Order_In_Making, default_initial_value=False)
    problem.add_fluent(Used_by, default_initial_value=False)
    problem.add_fluent(Distance, default_initial_value=0)
    problem.add_fluent(Holds, default_initial_value=False)
    problem.add_fluent(In_Rest, default_initial_value=False)

    if initial_state_dict is not None:

        cust_name_to_obj = {cust.name: cust for cust in customers}
        dish_name_to_obj = {dish.name: dish for dish in dishes}
        table_name_to_obj = {table.name: table for table in tables}
        loc_name_to_obj = {loc.name: loc for loc in locations}
        rob_name_to_obj = {rob.name: rob for rob in robots}

        problem.set_initial_value(Server_At(server_1, loc_name_to_obj[initial_state_dict['server_1']['position']]), True)
        problem.set_initial_value(Stood_In(loc_name_to_obj[initial_state_dict['server_1']['position']]), True)
        problem.set_initial_value(Holds(server_1), True if initial_state_dict['server_1']['holds'] is not None else False)

        for table_obj_ in tables:
            if initial_state_dict['server_1']['holds'] == table_obj_.name:
                problem.set_initial_value(Serves(server_1, table_obj_), True)

        problem.set_initial_value(Host_At(host, loc_name_to_obj[initial_state_dict['host']['position']]), True)
        problem.set_initial_value(Stood_In(loc_name_to_obj[initial_state_dict['host']['position']]), True)
        problem.set_initial_value(Seating_Customers(host), initial_state_dict['host']['seating_customers'])

        problem.set_initial_value(Cleaner_At(cleaner, loc_name_to_obj[initial_state_dict['cleaner']['position']]), True)
        problem.set_initial_value(Stood_In(loc_name_to_obj[initial_state_dict['cleaner']['position']]), True)
        # problem.set_initial_value(Cleaning(cleaner), initial_state_dict['cleaner']['cleaning'])
        problem.set_initial_value(Cleaning(cleaner), False)# <----

        for cust_name, cust_dict in initial_state_dict['customers'].items():
            cust_obj = cust_name_to_obj[cust_name]
            problem.set_initial_value(In_Rest(cust_obj), cust_dict['in_rest'])
            problem.set_initial_value(Seated(cust_obj), cust_dict['seated'])
            problem.set_initial_value(Following(host, cust_obj), cust_dict['following'])
            problem.set_initial_value(Eaten(cust_obj), cust_dict['eaten'])

            for table_obj_ in tables:
                if cust_dict['table'] == table_obj_.name:
                    problem.set_initial_value(Seated_At(cust_obj, table_obj_), True)
                else:
                    problem.set_initial_value(Seated_At(cust_obj, table_obj_), False)
            
            problem.set_initial_value(Party_Size(cust_obj), cust_dict['party_size'])

        for table_name, table_dict in initial_state_dict['orders'].items():
            table_obj = table_name_to_obj[table_name]
            problem.set_initial_value(Occupied(table_obj), table_dict['occupied'])
            problem.set_initial_value(Clean(table_obj), table_dict['clean'])
            problem.set_initial_value(Ready_To_Order(table_obj), table_dict['ready_to_order'])
            problem.set_initial_value(Order_Taken(table_obj), table_dict['order_taken'])
            problem.set_initial_value(Served(table_obj), table_dict['served'])
            problem.set_initial_value(Can_Be_Taken(table_obj), table_dict['can_be_taken'])
            problem.set_initial_value(Food_Order(table_obj), table_dict['food_order'])
            # problem.set_initial_value(Order_In_Making(table_obj), table_dict['in_making'])
            problem.set_initial_value(Order_In_Making(table_obj), False) # <----
            problem.set_initial_value(Cleaning_Time(table_obj), table_dict['cleaning_time'])
            problem.set_initial_value(Food_Prep_Time(table_obj), table_dict['food_prep_time'])
            problem.set_initial_value(Ordering_Time(table_obj), table_dict['pondering_time'])
            problem.set_initial_value(Eating_Time(table_obj), table_dict['eating_time'])


        for dish_name, dish_dict in initial_state_dict['dishes'].items():
            dish_obj = dish_name_to_obj[dish_name]
            problem.set_initial_value(Used(dish_obj), dish_dict['used'])
            for table_obj_ in tables:
                if dish_dict['used_by'] == table_obj_.name:
                    problem.set_initial_value(Used_by(dish_obj, table_obj_), True)
                else:
                    problem.set_initial_value(Used_by(dish_obj, table_obj_), False)

        for table_name, table_dict in initial_state_dict['tables'].items():
            table_obj = table_name_to_obj[table_name]
            problem.set_initial_value(Table_At(table_obj, loc_name_to_obj[table_dict['position']]), True)
            
        
        problem.set_initial_value(Revenue, initial_state_dict['revenue'])

    
    # printing Order_In_Making:

    print(problem.initial_values)


    # problem.add_objects([fake_customer, fake_table])

    problem.add_actions([pick_up_customers, seat_customers, clean_table, decide_order, take_order, make_food, take_food, serve_food, eat, host_move, cleaner_move, server_move, go_home])

    # if initial_state_dict is not None:
    #     parse_dict_to_problem(initial_state_dict, problem)

    if initial_state_dict is not None:
        for cust in customers:
            cur_table = None
            if not initial_state_dict['customers'][cust.name]['seated']:
                problem.add_goal(Seated(cust))
                continue
            else:
                for table_obj_ in tables:
                    if initial_state_dict['customers'][cust.name]['table'] == table_obj_.name:
                        cur_table = table_obj_

            if not initial_state_dict['orders'][cur_table.name]['order_taken']:
                problem.add_goal(Order_Taken(cur_table))
            # elif not initial_state_dict['orders'][cur_table.name]['served']:
            #     problem.add_goal(Served(cur_table))
            elif not initial_state_dict['customers'][cust.name]['eaten']:
                problem.add_goal(Eaten(cust))
        
    else:
        for cust in customers:
            problem.add_goal(Eaten(cust))

    
    # problem.add_goal(Holds(server_1))

    # MaximizeExpressionOnFinalState(Revenue)

    # Check for planners
    from unified_planning.shortcuts import get_environment

    env = get_environment()
    print(env.factory.engines)

    print(problem.goals)
    # Example heuristics
    heuristics = ['hmax', 'hadd', 'hff', 'blind', 'hlandmarks']

    tamer_planner_config = {
         'heuristic': 'hff',
         'weight': 0.8,
    }

    planner_config = {
    # 'time_limit': 300,  # Time limit in seconds
    # 'memory_limit': 1024,  # Memory limit in MB
    # 'heuristic': 'h_add',  # Heuristic to use
    # 'search_strategy': 'best_first',  # Search strategy
    # Add other parameters as needed
    # 'output_stream' : sys.stdout,
    }

    
    class TemporalSimulator:
        def __init__(self, problem, replanner):
            self.problem = problem
            self.state = problem.initial_value
            self.replanner = replanner

        def simulate(self, plan):
            events = []
            for start, action, duration in plan:
                if duration != None:
                    events.append((start, 'start', action))
                    events.append((start + duration, 'end', action))
                else:
                    events.append((start, 'start', action))
                    events.append((start, 'end', action))
                # start_time = action.start_time
                # end_time = start_time + action.duration
                # events.append((start_time, 'start', action))
                # events.append((end_time, 'end', action))

            events.sort(key=lambda x: x[0])

            for time, event, action in events:
                if event == 'start':
                    self.apply_start_effects(action)
                elif event == 'end':
                    self.apply_end_effects(action)

            return self.state, self.problem, self.replanner

        def apply_end_effects(self, action):
            # implemnt using update_initial_value
            if isinstance(action.action, DurativeAction):
                effect = action.action.effects[list(action.action.effects.keys())[0]]
                # print(effect[0].fluent, effect[0].value, effect[0].kind)
            if isinstance(action.action, InstantaneousAction):
                effect = action.action.effects

            

            parameters = action.actual_parameters
            # print(effect)
            # print(parameters)
            params_to_actual_params = dict()
            for param, ac_param in zip(action.action.parameters, action.actual_parameters):
                # print(param.name, ac_param)s
                params_to_actual_params[param.name] = ac_param
                # print(action.action.parameter(param.name))
            # apply the effects
            # print("paramters:", parameters)
            # print("params_to_actual_params:", params_to_actual_params)
            for fluent, value in zip([f.fluent for f in effect], [f.value for f in effect]):
                # print(fluent.args, params_to_actual_params)
                # print(type(fluent.args[0]))
                # print(str(fluent.args[0]))
                # print(type(fluent.args[0].parameter().name))
                cur_params = tuple([params_to_actual_params[arg.parameter().name] if arg.is_parameter_exp() else arg for arg in fluent.args])
                # print(cur_params)
                
                if not value.is_constant():
                    # print(value)
                    # print(value.get_contained_names())
                    # print("val_args:", value._content)
                    for name in value.get_contained_names():
                        if name in params_to_actual_params:
                            value = params_to_actual_params[name]
                            break

                # print(fluent.fluent()(*cur_params), value)
                # print(fluent.fluent(), fluent.args)
                # print(fluent.fluent()(params))
                self.problem.set_initial_value(fluent.fluent()(*cur_params), value)
                self.replanner.update_initial_value(fluent.fluent()(*cur_params), value)
            
        def apply_start_effects(self, action):
            # implemnt using update_initial_value
            # print(action, type(action))
            pass

    original_problem = problem
    # grounded_problem = ground_all_preconditions(original_problem)
    # with Compiler(problem_kind=problem.kind,
    # compilation_kind=CompilationKind.USERTYPE_FLUENTS_REMOVING, ) as compiler:
    #     compilation_result_1 = compiler.compile(problem)
    #     with Compiler(problem_kind=compilation_result_1.problem.kind, compilation_kind=CompilationKind.GROUNDING) as bounded_types_compiler:
    #         compilation_result_2 = bounded_types_compiler.compile(compilation_result_1.problem)

    start_time = time.time()
    action_sequence = []
    # compilation_result_1.problem.environment.factory.add_engine(name = "lpg", module_name ="up_lpg.lpg_planner", class_name = "LPGEngine")
    # for fluent in compilation_result_1.problem.fluents:
    #     print(fluent)
    # print(compilation_result_1.problem.goals)
    
    problem.add_quality_metric(MinimizeMakespan())

    # with AnytimePlanner(
    #     name = 'lpg-anytime',
    #     params= planner_config,
    #     problem_kind=problem.kind,
    #     anytime_guarantee="INCREASING_QUALITY"
    #     # problem_kind=problem.kind, anytime_guarantee="INCREASING_QUALITY"
    # ) as planner:
    #     for i, p in enumerate(planner.get_solutions(problem, output_stream = sys.stdout, timeout = 20)):
    #         plan = p.plan
    #         print(f"plan {i}")
    #         print(len(plan.timed_actions))
    #         for start, action, duration in plan.timed_actions:
    #             if duration != None:
    #                 print(f"{float(start)}: {action} [{float(duration)}]")
    #                 # start_t, end_t = _extract_action_timings(action.action, start=start, duration=duration)
    #                 # print(_get_timepoint_effects(action.action, start=start_t, timing=end_t, duration=duration))
    #             else:
    #                 print(f"{float(start)}: {action}")

    #         print("Done!")
    #         break


    with OneshotPlanner(problem_kind=problem.kind, name = 'lpg' , optimality_guarantee = PlanGenerationResultStatus.SOLVED_OPTIMALLY) as replanner:
        result = replanner.solve(problem)
        plan = result.plan
        # seudo_plan = result.plan
        # plan = seudo_plan.replace_action_instances(compilation_result_2.map_back_action_instance).replace_fluent_instances(compilation_result_1.map_back_fluent_instance)
        if plan is not None:
            print(f"{replanner.name} returned:")
            for start, action, duration in plan.timed_actions:
                if duration != None:
                    # print(f"{float(start)}: {action} [{float(duration)}]")
                    action_sequence.append({'action': action, 'start': float(start), 'duration': float(duration), 'is_instantaneous': 0})
                    # start_t, end_t = _extract_action_timings(action.action, start=start, duration=duration)
                    # print(_get_timepoint_effects(action.action, start=start_t, timing=end_t, duration=duration))
                else:
                    # print(f"{float(start)}: {action}")
                    action_sequence.append({'action': action, 'start': float(start), 'duration': 0,'is_instantaneous': 1})

            
        #     bias = action_sequence[-1]['start'] if action_sequence[-1]['is_instantaneous'] else action_sequence[-1]['start'] + action_sequence[-1]['duration']

        #     temporal_simulator = TemporalSimulator(problem, replanner)
        #     state, problem, replanner = temporal_simulator.simulate(plan.timed_actions)
        #     # replanner.add_goal(Served(customers[0]))
        #     # replanner.remove_goal(Holds(server_1))
        #     # replanner.add_goal(Clean(table_bl))
        #     cur_table = state(Seated_At(customers[0])) # table_bl
        #     table_loc = state(Table_At(cur_table))

        #     print(f"Table location: {table_loc}")
        #     replanner.add_goal(Eaten(customers[1]))
        #     replanner.add_goal(Clean(cur_table))
        #     replanner.add_goal(Seated(customers[2]))
        #     replanner.add_goal(Occupied(table_tr))
        #     # replanner.add_goal(Holds(server_1))

        #     new_result = replanner.resolve()
        #     new_plan = new_result.plan
        #     if new_plan is not None:
        #         print(f"{replanner.name} returned:")
        #         for start, action, duration in new_plan.timed_actions:
        #             if duration != None:
        #                 print(f"{float(bias+start)}: {action} [{float(duration)}]")
        #                 action_sequence.append({'action': action, 'start': float(bias+start), 'duration': float(duration), 'is_instantaneous': 0})
        #             else:
        #                 print(f"{float(bias+start)}: {action}")
        #                 action_sequence.append({'action': action, 'start': float(bias+start), 'is_instantaneous': 1})

        #     else:
        #         print(f"{replanner.name} failed to find a plan")

        else:
            print(f"{replanner.name} failed to find a plan")

    print(f"Time taken: {time.time() - start_time}")
    action_sequence = sorted(action_sequence, key=lambda x: (x['start'], -x['is_instantaneous']))
    # for action in action_sequence:
    #     print(action['start'], action['action'], action['duration'])

    print("Done")

    return action_sequence


    
if __name__ == '__main__':
    solve_problem(None)