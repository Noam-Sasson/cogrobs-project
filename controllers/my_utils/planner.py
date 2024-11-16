from unified_planning.shortcuts import *
from unified_planning.model import Problem, Fluent, Action, Object
from unified_planning.engines import engine
from unified_planning.engines.sequential_simulator import UPSequentialSimulator
from unified_planning.model.metrics import MaximizeExpressionOnFinalState
from unified_planning.plans.plan import PlanKind
from unified_planning.engines import PlanGenerationResultStatus
from collections import defaultdict
from itertools import product
import time
from unified_planning.plans.time_triggered_plan import _get_timepoint_effects, _extract_action_timings, _get_timepoint_simulated_effects


def solve_problem(initial_state_dict : dict):
    '''
    initial_state_dict should countain the following:
    - server_1_cur_loc: server_1 current location
    - cleaer_cur_loc: cleaner current location
    - host_cur_loc: host current location
    - g_following: group following the host
    - orders: orders status of the form {order_id: {customer_id, table_id}}
    - cust_out: list of customers that are outsid of the restaurant
    - cust_in: customers iniside status of the form {customer_id: {table_id, seated, ready_to_order, order_taken, served, eaten, party_size}}
    - tables: tables status of the form {table_id: {occupied, clean}}
    - revenue: current revenue
    '''
    # Create the planning problem
    CUST_COUNT = 5
    DISH_COUNT = 2
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

    tables = [
        table_tl := Object('table_tl', Table),
        table_bl := Object('table_bl', Table),
        table_tr := Object('table_tr', Table),
        table_br := Object('table_br', Table),
    ]
    # fake_table = Object('fake_table', Table)
    locations = [
        (kitchen := Object("kitchen", Location)),
        (TL1 := Object('TL1', Location)),
        (TL2 := Object('TL2', Location)),
        (ML1 := Object('ML1', Location)),
        (ML2 := Object('ML2', Location)),
        (BL1 := Object('BL1', Location)),
        (BL2 := Object('BL2', Location)),
        (TR1 := Object('TR1', Location)),
        (TR2 := Object('TR2', Location)),
        (MR1 := Object('MR1', Location)),
        (MR2 := Object('MR2', Location)),
        (BR1 := Object('BR1', Location)),
        (BR2 := Object('BR2', Location)),
        (ML := Object('ML', Location)),
        (MR := Object('MR', Location)),
        (TBL_TL := Object('TBL_TL', Location)),
        (TBL_BL := Object('TBL_BL', Location)),
        (TBL_TR := Object('TBL_TR', Location)),
        (TBL_BR := Object('TBL_BR', Location)),
        (K := Object('K', Location)),
        (fake_location := Object('fake_location', Location))
    ]
    robots = [
        host := Object("host", Host),
        server_1 := Object("server_1", Server),
        # server_2 := Object("server_2", Robot),
        cleaner := Object("cleaner", Cleaner)

    ]
    customers = [Object(f"customer_{i}", Customer) for i in range(1, CUST_COUNT+1)]
    # fake_customer = Object('fake_customer', Customer)
    # orders = [Object(f'order_{table}', Order) for table in tables]
    # fake_order = Object('fake_order', Order)
    dishes = [Object(f'dish_{i}', Dish) for i in range(1, DISH_COUNT+1)]
    problem.add_objects(locations + tables + robots + customers + dishes)
    # Predicates
    Adjacent = Fluent("Adjacent", BoolType(), loc1=Location,
                      loc2=Location)  # location1 is adjacent to location2
    Distance = Fluent("Distance", IntType(), loc1=Location, loc2=Location)
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

    '''
    actual ground distances
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

    for customer in customers:
        problem.set_initial_value(Party_Size(customer), 2)

    Holds = Fluent('Holds', BoolType(), server = Server)
    Serves = Fluent('serves', BoolType(), server = Server, table =Table)

    # Owner = Fluent('Owner', Customer, order=Order)
    Food_Order = Fluent('Food', IntType(), table = Table)
    Food_Prep_Time = Fluent('Food_Prep_Time', IntType(), table = Table)
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

    clean_table.set_fixed_duration(CLEANING_TIME := 60)  # TODO: Decide on Cleaning time
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

    decide_order.set_fixed_duration(ORDERING_TIME := 60)  # TODO: Decide on Ordering time
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
    take_order.add_effect(Food_Order(table), 20)#Times(Party_Size(cust), 20)) to be handled by simulation

    # prepare the food
    make_food = DurativeAction(f'make_order', table = Table, dish=Dish)
    
    # order = make_food.parameter('order')
    table = make_food.parameter('table')
    make_food.set_fixed_duration(total_time := Food_Prep_Time(table))#Times(Party_Size(Owner(order)), PREP_TIME))

    make_food.add_condition(StartTiming(), Used_by(dish, table))
    make_food.add_condition(StartTiming(), GT(Food_Order(table), 0)) # order has food
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

    eat.set_fixed_duration(EATING_TIME := 120)
    eat.add_condition(StartTiming(), Served(table))
    eat.add_condition(StartTiming(), Seated_At(cust, table))

    eat.add_effect(EndTiming(), Occupied(table), False)
    eat.add_effect(EndTiming(), Clean(table), False)
    eat.add_effect(EndTiming(), Eaten(cust), True)
    eat.add_increase_effect(EndTiming(), Revenue, 20) #Times(Party_Size(cust), 20)) ## will be insreted by simulation
    
    eat.add_effect(EndTiming(), Served(table), False)
    eat.add_effect(EndTiming(), Ready_To_Order(table), False)
    eat.add_effect(EndTiming(), Order_Taken(table), False)
    eat.add_effect(EndTiming(), Food_Order(table), 0)
    eat.add_effect(EndTiming(), Seated_At(cust, table), False)

    host_move = DurativeAction('host_move', rob=Host, loc1=Location, loc2=Location)
    rob = host_move.parameter('rob')
    loc1 = host_move.parameter('loc1')
    loc2 = host_move.parameter('loc2')
    host_move.set_fixed_duration(Distance(loc1, loc2))

    host_move.add_condition(StartTiming(), Adjacent(loc1, loc2))
    host_move.add_condition(StartTiming(), Host_At(rob, loc1))
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

    go_home = DurativeAction('go_home', loc1 = Location ,rob=Host)
    rob = go_home.parameter('rob')
    loc1 = go_home.parameter('loc1')
    go_home.set_fixed_duration(9) # TODO: Decide on going home time

    go_home.add_condition(EndTiming(), Not(Stood_In(TL1)))
    go_home.add_condition(StartTiming(), Host_At(rob, loc1))
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

    for dish in dishes:
        problem.set_initial_value(Used(dish), False)

    # for customer in customers+[fake_customer]:
    #     problem.set_initial_value(Seated_At(customer), fake_table)

    problem.add_fluent(Cleaning, default_initial_value=False)
    # problem.set_initial_value(Occupied(fake_table), True)
    problem.add_fluent(Stood_In, default_initial_value=False)
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
    problem.add_fluent(Food_Prep_Time, default_initial_value=320) # 3 minutes
    problem.add_fluent(In_Rest, default_initial_value=False)

    # problem.add_objects([fake_customer, fake_table])

    problem.add_actions([pick_up_customers, seat_customers, clean_table, decide_order, take_order, make_food, take_food, serve_food, eat, host_move, cleaner_move, server_move, go_home])

    problem.add_quality_metric(MinimizeMakespan())
    # if initial_state_dict is not None:
    #     parse_dict_to_problem(initial_state_dict, problem)

    for cust in customers:
        problem.add_goal(Eaten(cust))
    
    # problem.add_goal(Holds(server_1))

    # MaximizeExpressionOnFinalState(Revenue)

    # Check for planners
    from unified_planning.shortcuts import get_environment

    env = get_environment()
    print(env.factory.engines)
    # Example heuristics
    heuristics = ['hmax', 'hadd', 'hff', 'blind', 'hlandmarks']

    planner_config = {
         'heuristic': 'hff',
         'weight': 0.8,
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

    def convert_durative_to_instantaneous(problem: Problem) -> Problem:
        new_problem = Problem(problem.name + "_instantaneous")
        
        # Copy fluents, objects, and initial state
        for fluent in problem.fluents:
            new_problem.add_fluent(fluent)
        for obj in problem.objects:
            new_problem.add_object(obj)
        for fluent, value in problem.initial_values.items():
            new_problem.set_initial_value(fluent, value)
        
        # Copy goals
        for goal in problem.goals:
            new_problem.add_goal(goal)
        
        # Convert durative actions to instantaneous actions
        for action in problem.actions:
            if isinstance(action, DurativeAction):
                # Split durative action into start and end instantaneous actions
                start_action = InstantaneousAction(action.name + "_start")
                end_action = InstantaneousAction(action.name + "_end")
                
                # Copy parameters
                for param in action.parameters:
                    start_action.parameters.append(param)
                    end_action.parameters.append(param)
                
                # Copy conditions and effects
                for condition in action.conditions:
                    if condition.timing.is_start():
                        start_action.add_precondition(condition.condition)
                    elif condition.timing.is_end():
                        end_action.add_precondition(condition.condition)
                for effect in action.effects:
                    if effect.timing.is_start():
                        start_action.add_effect(effect.fluent, effect.value)
                    elif effect.timing.is_end():
                        end_action.add_effect(effect.fluent, effect.value)
                
                new_problem.add_action(start_action)
                new_problem.add_action(end_action)
            else:
                new_problem.add_action(action)
        
        return new_problem
   
    def ground_preconditions(preconditions, problem):
        grounded_preconditions = []
        for precondition in preconditions:
            if precondition.is_exists():
                # Ground the existential quantifier by iterating over all possible values
                for typename in problem.user_types:
                    for obj in problem.objects(typename):
                        grounded_precondition = precondition.ground({precondition.variable: obj})
                        grounded_preconditions.append(grounded_precondition)
            else:
                grounded_preconditions.append(precondition)
        return grounded_preconditions

    def ground_all_preconditions(problem: Problem) -> Problem:
        new_problem = Problem(problem.name + "_grounded")
    
        # Copy fluents, objects, and initial state
        for fluent in problem.fluents:
            new_problem.add_fluent(fluent)
        for typename in problem.user_types:
            for obj in problem.objects(typename):
                new_problem.add_object(obj)
        for fluent, value in problem.initial_values.items():
            new_problem.set_initial_value(fluent, value)
        
        # Copy goals
        for goal in problem.goals:
            new_problem.add_goal(goal)
        
        # Ground preconditions for all actions
        for action in problem.actions:
            if isinstance(action, DurativeAction):
                new_action = DurativeAction(action.name)
                # Copy parameters
                for param in action.parameters:
                    new_action.parameters.append(param)
                # Ground preconditions
                start_preconditions = [cond.condition for cond in action.conditions if isinstance(cond.timing, StartTiming)]
                end_preconditions = [cond.condition for cond in action.conditions if isinstance(cond.timing, EndTiming)]
                for precondition in ground_preconditions(start_preconditions, problem):
                    new_action.add_start_precondition(precondition)
                for precondition in ground_preconditions(end_preconditions, problem):
                    new_action.add_end_precondition(precondition)
                # Copy effects
                for effect in action.effects:
                    new_action.add_effect(effect.fluent, effect.value, effect.timing)
                new_problem.add_action(new_action)
            elif isinstance(action, InstantaneousAction):
                new_action = InstantaneousAction(action.name)
                # Copy parameters
                for param in action.parameters:
                    new_action.parameters.append(param)
                # Ground preconditions
                for precondition in ground_preconditions(action.preconditions, problem):
                    new_action.add_precondition(precondition)
                # Copy effects
                for effect in action.effects:
                    new_action.add_effect(effect.fluent, effect.value)
                new_problem.add_action(new_action)
        
        return new_problem
    
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
    
    with Replanner(problem=problem, name="replanner[lpg]", optimality_guarantee=PlanGenerationResultStatus.SOLVED_OPTIMALLY) as replanner:
        result = replanner.resolve()
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
                    action_sequence.append({'action': action, 'start': float(start), 'is_instantaneous': 1})

            
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
    for action in action_sequence:
        print(action['start'], action['action'])

    print("Done")

    return action_sequence

    def parse_dict_to_problem(initial_state_dict, problem):
        '''
        initial_state_dict should countain the following:
        - server_1_cur_loc: server_1 current location
        - cleaer_cur_loc: cleaner current location
        - host_cur_loc: host current location
        - g_following: group following the host
        - holds : order_id
        - orders: orders status of the form {order_id: {customer_id, table_id, profit, prep_time}}
        - cust_out: list of customers that are outsid of the restaurant
        - cust_in: customers iniside status of the form {customer_id: {table_id, seated, ready_to_order, order_taken, served, eaten, party_size}}
        - tables: tables status of the form {table_id: {occupied, clean}}
        - revenue: current revenue
        '''

        # holds and tables and more need to be added

        problem.set_initial_value(At(server_1), initial_state_dict['server_1_cur_loc'])
        problem.set_initial_value(At(cleaner), initial_state_dict['cleaner_cur_loc'])
        problem.set_initial_value(At(host), initial_state_dict['host_cur_loc'])
        # set other groups not following the host
        problem.set_initial_value(Following(host, initial_state_dict['g_following']), True)
        for order_id, order in initial_state_dict['orders'].items():
            problem.set_initial_value(Owner(orders[order_id]), customers[order['customer_id']])
            problem.set_initial_value(Seated_At(customers[order['customer_id']]), tables[order['table_id']])
            problem.set_initial_value(Used(orders[order_id]), True)
            problem.set_initial_value(Food(orders[order_id]), order['profit'])
            problem.set_initial_value(Food_Prep_Time(orders[order_id]), order['prep_time'])
        for customer_id, customer in initial_state_dict['cust_in'].items():
            problem.set_initial_value(Seated(customers[customer_id]), customer['seated'])
            problem.set_initial_value(Ready_To_Order(customers[customer_id]), customer['ready_to_order'])
            problem.set_initial_value(Order_Taken(customers[customer_id]), customer['order_taken'])
            problem.set_initial_value(Served(customers[customer_id]), customer['served'])
            problem.set_initial_value(Eaten(customers[customer_id]), customer['eaten'])
            problem.set_initial_value(Party_Size(customers[customer_id]), customer['party_size'])
        for table_id, table in initial_state_dict['tables'].items():
            problem.set_initial_value(Occupied(tables[table_id]), table['occupied'])
            problem.set_initial_value(Clean(tables[table_id]), table['clean'])
        problem.set_initial_value(Revenue, initial_state_dict['revenue'])
        problem.set_initial_value(Holds(server_1), initial_state_dict['holds'])
        


    
if __name__ == '__main__':
    solve_problem(None)