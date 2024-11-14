from unified_planning.shortcuts import *
from unified_planning.model import Problem, Fluent, Action, Object
from unified_planning.engines import engine
from unified_planning.engines.sequential_simulator import UPSequentialSimulator
from unified_planning.model.metrics import MaximizeExpressionOnFinalState
from unified_planning.plans.plan import PlanKind
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
    CUST_COUNT = 2
    problem = Problem("DinerProblem")

    # Define types of objects (Table, Robot, Customer)
    Table = UserType("Table")
    Robot = UserType("Robot")
    Customer = UserType("Customer")
    Location = UserType("Location")
    Order = UserType('Order')
    # Define objects in the problem

    tables = [
        table_tl := Object('table_tl', Table),
        table_bl := Object('table_bl', Table),
        table_tr := Object('table_tr', Table),
        table_br := Object('table_br', Table),
    ]
    fake_table = Object('fake_table', Table)
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
        host := Object("host", Robot),
        server_1 := Object("server_1", Robot),
        # server_2 := Object("server_2", Robot),
        cleaner := Object("cleaner", Robot)

    ]
    customers = [Object(f"customer_{i}", Customer) for i in range(1, CUST_COUNT+1)]
    fake_customer = Object('fake_customer', Customer)
    orders = [Object(f'order{i}', Order) for i in range(2)]
    fake_order = Object('fake_order', Order)
    problem.add_objects(locations + tables + robots + customers + orders)

    # Predicates
    Adjacent = Fluent("Adjacent", BoolType(), location1=Location,
                      location2=Location)  # location1 is adjacent to location2
    Distance = Fluent("Distance", IntType(), location1=Location, location2=Location)
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

    Job = Fluent('Job', IntType(), robot=Robot)
    IsAerial = Fluent('IsAerial', BoolType(), robot=Robot)
    HOST, SERVER, CLEANER = 0, 1, 2
    # Fluents
    At = Fluent('At', Location, robot=Robot)
    Table_At = Fluent('Table_At', Location, table=Table)

    Occupied = Fluent("Occupied", BoolType(), table=Table)
    Clean = Fluent("Clean", BoolType(), table=Table)

    Seating_Customers = Fluent('Seating_Customer', BoolType(), robot=Robot)
    Following = Fluent('Following', BoolType(), robot=Robot, customer=Customer)

    Seated = Fluent('Seated', BoolType(), customer=Customer)
    Ready_To_Order = Fluent('Ready_To_Order', BoolType(), customer=Customer)
    Order_Taken = Fluent('Order_Taken', BoolType(), customer=Customer)
    Served = Fluent("Served", BoolType(), customer=Customer)
    Eaten = Fluent("Eaten", BoolType(), customer=Customer)
    Seated_At = Fluent('Seated_At', Table, customer=Customer)
    Party_Size = Fluent('Party_Size', IntType(), customer=Customer)

    for customer in customers:
        problem.set_initial_value(Party_Size(customer), 2)

    Holds = Fluent('Holds', BoolType(), robot=Robot)
    Serves = Fluent('serves', Order, robot=Robot)

    Used = Fluent('Used', BoolType(), order=Order)
    Owner = Fluent('Owner', Customer, order=Order)
    Food = Fluent('Food', IntType(), order=Order)
    Food_Prep_Time = Fluent('Food_Prep_Time', IntType(), order=Order)
    Done = Fluent('Done', BoolType(), order=Order)

    Stood_In = Fluent('Stood_In', BoolType(), location=Location)

    Revenue = Fluent('Revenue', IntType())
    problem.add_fluent(Revenue, default_initial_value=0)

    # Define actions
    # pick up customers
    pick_up_customers = InstantaneousAction('take_customers', rob=Robot, cust=Customer)
    rob = pick_up_customers.parameter('rob')
    cust = pick_up_customers.parameter('cust')

    pick_up_customers.add_precondition(Equals(Job(rob), HOST))
    pick_up_customers.add_precondition(Equals(At(rob), TL1))
    pick_up_customers.add_precondition(Not(Seating_Customers(rob)))

    pick_up_customers.add_effect(Seating_Customers(rob), True)
    pick_up_customers.add_effect(Following(rob, cust), True)

    # seat customers
    seat_customers = InstantaneousAction('seat_customers', rob=Robot, table=Table, cust=Customer)
    rob = seat_customers.parameter('rob')
    table = seat_customers.parameter('table')
    cust = seat_customers.parameter('cust')

    seat_customers.add_precondition(Seating_Customers(rob))
    seat_customers.add_precondition(Following(rob, cust))
    seat_customers.add_precondition(Equals(At(rob), Table_At(table)))
    seat_customers.add_precondition(Not(Occupied(table)))
    seat_customers.add_precondition(Clean(table))

    seat_customers.add_effect(Seating_Customers(rob), False)
    seat_customers.add_effect(Following(rob, cust), False)

    seat_customers.add_effect(Occupied(table), True)
    seat_customers.add_effect(Seated_At(cust), table)
    seat_customers.add_effect(Seated(cust), True)

    # clean a table
    clean_table = DurativeAction('clean_table', rob=Robot, table=Table)
    rob = clean_table.parameter('rob')
    table = clean_table.parameter('table')

    clean_table.set_fixed_duration(CLEANING_TIME := 2)  # TODO: Decide on Cleaning time
    clean_table.add_condition(StartTiming(), Equals(Job(rob), CLEANER))
    clean_table.add_condition(StartTiming(), Equals(At(rob), Table_At(table)))
    clean_table.add_condition(StartTiming(), Not(Occupied(table)))
    clean_table.add_condition(StartTiming(), Not(Clean(table)))

    clean_table.add_effect(EndTiming(), Clean(table), True)

    # decide order
    decide_order = DurativeAction('decide_order', cust=Customer)
    cust = decide_order.parameter('cust')

    decide_order.set_fixed_duration(ORDERING_TIME := 2)  # TODO: Decide on Ordering time
    decide_order.add_condition(StartTiming(), Seated(cust))
    decide_order.add_condition(StartTiming(), Not(Ready_To_Order(cust)))
    decide_order.add_condition(StartTiming(), Not(Order_Taken(cust)))
    decide_order.add_condition(StartTiming(), Not(Served(cust)))

    decide_order.add_effect(EndTiming(), Ready_To_Order(cust), True)

    # actually order
    take_order = InstantaneousAction(f'take_order', rob=Robot, cust=Customer, table=Table, order=Order)
    rob = take_order.parameter('rob')
    cust = take_order.parameter('cust')
    table = take_order.parameter('table')
    order = take_order.parameter('order')

    take_order.add_precondition(Equals(Job(rob), SERVER))
    take_order.add_precondition(Ready_To_Order(cust))
    take_order.add_precondition(Equals(Seated_At(cust), table))
    take_order.add_precondition(Equals(At(rob), Table_At(table)))
    take_order.add_precondition(Not(Order_Taken(cust)))
    take_order.add_precondition(Not(Served(cust)))
    take_order.add_precondition(Not(Used(order)))

    take_order.add_effect(Order_Taken(cust), True)
    take_order.add_effect(Used(order), True)
    take_order.add_effect(Owner(order), cust)
    take_order.add_effect(Food(order), 20)#Times(Party_Size(cust), 20)) to be handled by simulation

    # prepare the food
    make_food = DurativeAction(f'make_order', order=Order)
    PREP_TIME = 2  # TODO: Decide on Prepping time

    order = make_food.parameter('order')
    make_food.set_fixed_duration(total_time := Food_Prep_Time(order))#Times(Party_Size(Owner(order)), PREP_TIME))

    make_food.add_condition(StartTiming(), Used(order))
    make_food.add_condition(StartTiming(), Not(Done(order)))

    make_food.add_effect(EndTiming(), Done(order), True)

    # take food
    take_food = InstantaneousAction('take_food', rob=Robot, order=Order)
    rob = take_food.parameter('rob')
    order = take_food.parameter('order')

    take_food.add_precondition(Equals(Job(rob), SERVER))
    take_food.add_precondition(Not(Holds(rob)))
    take_food.add_precondition(Equals(At(rob), kitchen))
    take_food.add_precondition(Done(order))

    take_food.add_effect(Holds(rob), True)
    take_food.add_effect(Serves(rob), order)
    take_food.add_effect(Done(order), False)
    take_food.add_effect(Used(order), False)

    # give food
    serve_food = InstantaneousAction('serve_food', rob=Robot, cust=Customer)
    rob = serve_food.parameter('rob')
    cust = serve_food.parameter('cust')

    serve_food.add_precondition(Holds(rob))
    serve_food.add_precondition(Equals(At(rob), Table_At(Seated_At(cust))))
    serve_food.add_precondition(Equals(Owner(Serves(rob)), cust))

    serve_food.add_effect(Holds(rob), False)
    serve_food.add_effect(Served(cust), True)

    # eat food
    eat = DurativeAction('eat', cust=Customer, table=Table)
    cust = eat.parameter('cust')
    table = eat.parameter('table')

    eat.set_fixed_duration(EATING_TIME := 10)
    eat.add_condition(StartTiming(), Served(cust))
    eat.add_condition(StartTiming(), Equals(Seated_At(cust), table))

    eat.add_effect(EndTiming(), Occupied(table), False)
    eat.add_effect(EndTiming(), Clean(table), False)
    eat.add_effect(EndTiming(), Eaten(cust), True)
    eat.add_increase_effect(EndTiming(), Revenue, 20)#Times(Party_Size(cust), 20)) ## will be insreted by simulation

    move = DurativeAction('move', rob=Robot, loc1=Location, loc2=Location)
    rob = move.parameter('rob')
    loc1 = move.parameter('loc1')
    loc2 = move.parameter('loc2')
    move.set_fixed_duration(Distance(loc1, loc2))

    move.add_condition(StartTiming(), Adjacent(loc1, loc2))
    move.add_condition(StartTiming(), Equals(At(rob), loc1))
    move.add_condition(StartTiming(), Not(Stood_In(loc2)))

    move.add_effect(EndTiming(), Stood_In(loc1), False)
    move.add_effect(EndTiming(), At(rob), loc2)
    move.add_effect(EndTiming(), Stood_In(loc2), True)

    go_home = DurativeAction('go_home', rob=Robot)
    rob = go_home.parameter('rob')
    go_home.set_fixed_duration(9) # TODO: Decide on going home time

    go_home.add_condition(StartTiming(), Not(Stood_In(TL1)))
    go_home.add_condition(StartTiming(), IsAerial(rob))
    go_home.add_condition(StartTiming(), Not(Seating_Customers(rob)))

    go_home.add_effect(StartTiming(), Stood_In(TL1), False)
    go_home.add_effect(EndTiming(), At(rob), TL1)
    go_home.add_effect(EndTiming(), Stood_In(TL1), True)

    # Initial state
    problem.add_fluents([At, Table_At, Job, Adjacent, Owner, Serves, Seated_At, IsAerial])
    for order in orders+[fake_order]:
        problem.set_initial_value(Owner(order), fake_customer)
    for rob in robots:
        problem.set_initial_value(Serves(rob), fake_order)
        problem.set_initial_value(IsAerial(rob), False)
    for customer in customers+[fake_customer]:
        problem.set_initial_value(Seated_At(customer), fake_table)

    problem.set_initial_value(Used(fake_order), True)
    problem.add_fluent(Stood_In, default_initial_value=False)
    problem.set_initial_value(At(host), TL1)  # Host starts at the entrance
    problem.set_initial_value(IsAerial(host), True)
    problem.set_initial_value(Stood_In(TL1), True)
    problem.set_initial_value(At(server_1), K)  # Server starts at k
    # problem.set_initial_value(At(server_2), BL2)  # Server starts at k
    # problem.set_initial_value(Stood_In(BL2), True)
    problem.set_initial_value(Stood_In(K), True)
    problem.set_initial_value(At(cleaner), kitchen)  # Cleaner starts at the kitchen
    problem.set_initial_value(Stood_In(kitchen), True)
    problem.set_initial_value(Table_At(table_bl), TBL_BL)
    problem.set_initial_value(Table_At(table_tl), TBL_TL)
    problem.set_initial_value(Table_At(table_tr), TBL_TR)
    problem.set_initial_value(Table_At(table_br), TBL_BR)
    problem.set_initial_value(Job(host), HOST)
    problem.set_initial_value(Job(server_1), SERVER)
    # problem.set_initial_value(Job(server_2), SERVER)
    problem.set_initial_value(Job(cleaner), CLEANER)
    problem.set_initial_value(Table_At(fake_table), fake_location)

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
    problem.add_fluent(Done, default_initial_value=False)
    problem.add_fluent(Food, default_initial_value=0)
    problem.add_fluent(Distance, default_initial_value=0)
    problem.add_fluent(Holds, default_initial_value=False)
    problem.add_fluent(Food_Prep_Time, default_initial_value=2)

    problem.add_objects([fake_customer, fake_order, fake_table])

    problem.add_actions([pick_up_customers, seat_customers, clean_table, decide_order, take_order, make_food, take_food, serve_food, eat, move, go_home])


    # if initial_state_dict is not None:
    #     parse_dict_to_problem(initial_state_dict, problem)

    for cust in customers:
        problem.add_goal(Seated(cust))
        problem.add_goal(Order_Taken(cust))
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
    # with Compiler(problem_kind=grounded_problem.kind,
    # compilation_kind=CompilationKind.USERTYPE_FLUENTS_REMOVING, ) as compiler:
    #     compilation_result_1 = compiler.compile(grounded_problem)
    #     with Compiler(problem_kind=compilation_result_1.problem.kind, compilation_kind=CompilationKind.GROUNDING) as bounded_types_compiler:
    #         compilation_result_2 = bounded_types_compiler.compile(compilation_result_1.problem)

    start_time = time.time()
    action_sequence = []
    with Replanner(problem=problem, name="replanner[tamer]", params=planner_config) as replanner:
        result = replanner.resolve(problem)
        plan = result.plan
        # seudo_plan = result.plan
        # plan = seudo_plan.replace_action_instances(compilation_result_2.map_back_action_instance).replace_fluent_instances(compilation_result_1.map_back_fluent_instance)
        if plan is not None:
            print(f"{replanner.name} returned:")
            for start, action, duration in plan.timed_actions:
                if duration != None:
                    print(f"{float(start)}: {action} [{float(duration)}]")
                    action_sequence.append({'action': action, 'start': float(start), 'duration': float(duration), 'is_instantaneous': 0})
                    # start_t, end_t = _extract_action_timings(action.action, start=start, duration=duration)
                    # print(_get_timepoint_effects(action.action, start=start_t, timing=end_t, duration=duration))
                else:
                    print(f"{float(start)}: {action}")
                    action_sequence.append({'action': action, 'start': float(start), 'is_instantaneous': 1})

            
            bias = action_sequence[-1]['start'] if action_sequence[-1]['is_instantaneous'] else action_sequence[-1]['start'] + action_sequence[-1]['duration']

            temporal_simulator = TemporalSimulator(problem, replanner)
            state, problem, replanner = temporal_simulator.simulate(plan.timed_actions)
            replanner.add_goal(Served(customers[0]))
            replanner.remove_goal(Holds(server_1))
            # replanner.add_goal(Clean(table_bl))
            cur_table = state(Seated_At(customers[0])) # table_bl
            table_loc = state(Table_At(cur_table))

            print(f"Table location: {table_loc}")
            replanner.add_goal(Eaten(customers[0]))
            replanner.add_goal(Equals(At(cleaner), table_loc))
            replanner.add_goal(Clean(cur_table))
            # replanner.add_goal(Holds(server_1))

            new_result = replanner.resolve()
            new_plan = new_result.plan
            if new_plan is not None:
                print(f"{replanner.name} returned:")
                for start, action, duration in new_plan.timed_actions:
                    if duration != None:
                        print(f"{float(bias+start)}: {action} [{float(duration)}]")
                        action_sequence.append({'action': action, 'start': float(bias+start), 'duration': float(duration), 'is_instantaneous': 0})
                    else:
                        print(f"{float(bias+start)}: {action}")
                        action_sequence.append({'action': action, 'start': float(bias+start), 'is_instantaneous': 1})

            else:
                print(f"{replanner.name} failed to find a plan")

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