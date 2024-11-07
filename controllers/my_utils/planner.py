from unified_planning.shortcuts import *
from unified_planning.model import Problem, Fluent, Action, Object
from unified_planning.engines import engine

from unified_planning.model.metrics import MaximizeExpressionOnFinalState
from collections import defaultdict
from itertools import product


def create_diner_problem():
    # Create the planning problem
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
        server_2 := Object("server_2", Robot),
        cleaner := Object("cleaner", Robot)

    ]
    customers = [Object(f"customer_{i}", Customer) for i in range(1, 5)]
    fake_customer = Object('fake_customer', Customer)
    orders = [Object(f'order{i}', Order) for i in range(2)]
    fake_order = Object('fake_order', Order)
    problem.add_objects(locations + tables + robots + customers + orders)

    # Predicates
    Adjacent = Fluent("Adjacent", BoolType(), location1=Location,
                      location2=Location)  # location1 is adjacent to location2
    Distance = Fluent("Distance", IntType(), location1=Location, location2=Location)
    Distances = {
        (kitchen, K): 1,
        (K, TR1): 1,
        (K, TR2): 1,
        (TL2, TL1): 1,
        (TL1, ML1): 1,
        (ML1, BL1): 1,
        (BL1, BL2): 1,
        (BL2, ML2): 1,
        (ML2, TL2): 1,
        (ML, ML1): 1,
        (ML, ML2): 1,
        (ML, TBL_TL): 1,
        (ML, TBL_BL): 1,
        (TR1, MR1): 1,
        (MR1, BR1): 1,
        (BR1, BR2): 1,
        (BR2, MR2): 1,
        (MR2, TR2): 1,
        (MR, MR1): 1,
        (MR, MR2): 1,
        (MR, TBL_TR): 1,
        (MR, TBL_BR): 1,
        (ML2, MR1): 1,
        (BL2, BR1): 1
    }
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
    Seated_At = Fluent('Seated_At', Table, customer=Customer)
    Party_Size = Fluent('Party_Size', IntType(), customer=Customer)

    for customer in customers:
        problem.set_initial_value(Party_Size(customer), 2)

    Holds = Fluent('Holds', BoolType(), robot=Robot)
    Serves = Fluent('serves', Order, robot=Robot)

    Used = Fluent('Used', BoolType(), order=Order)
    Owner = Fluent('Owner', Customer, order=Order)
    Food = Fluent('Food', IntType(), order=Order)
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
    take_order.add_effect(Food(order), Times(Party_Size(cust), 20))

    # prepare the food
    make_food = DurativeAction(f'make_order', order=Order)
    PREP_TIME = 2  # TODO: Decide on Prepping time

    order = make_food.parameter('order')
    make_food.set_fixed_duration(total_time := Times(Party_Size(Owner(order)), PREP_TIME))

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
    eat.add_increase_effect(EndTiming(), Revenue, Times(Party_Size(cust), 20))

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

    # Initial state
    problem.add_fluents([At, Table_At, Job, Adjacent, Owner, Serves, Seated_At])
    for order in orders+[fake_order]:
        problem.set_initial_value(Owner(order), fake_customer)
    for rob in robots:
        problem.set_initial_value(Serves(rob), fake_order)
    for customer in customers+[fake_customer]:
        problem.set_initial_value(Seated_At(customer), fake_table)

    problem.add_fluent(Stood_In, default_initial_value=False)
    problem.set_initial_value(At(host), TL1)  # Host starts at the entrance
    problem.set_initial_value(Stood_In(TL1), True)
    problem.set_initial_value(At(server_1), K)  # Server starts at k
    problem.set_initial_value(At(server_2), BL2)  # Server starts at k
    problem.set_initial_value(Stood_In(BL2), True)
    problem.set_initial_value(Stood_In(K), True)
    problem.set_initial_value(At(cleaner), kitchen)  # Cleaner starts at the kitchen
    problem.set_initial_value(Stood_In(kitchen), True)
    problem.set_initial_value(Table_At(table_bl), TBL_BL)
    problem.set_initial_value(Table_At(table_tl), TBL_TL)
    problem.set_initial_value(Table_At(table_tr), TBL_TR)
    problem.set_initial_value(Table_At(table_br), TBL_BR)
    problem.set_initial_value(Job(host), HOST)
    problem.set_initial_value(Job(server_1), SERVER)
    problem.set_initial_value(Job(server_2), SERVER)
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

    problem.add_fluent(Used, default_initial_value=False)
    problem.add_fluent(Done, default_initial_value=False)
    problem.add_fluent(Food, default_initial_value=0)
    problem.add_fluent(Distance, default_initial_value=0)
    problem.add_fluent(Holds, default_initial_value=False)

    problem.add_objects([fake_customer, fake_order, fake_table])

    problem.add_actions([pick_up_customers, seat_customers, clean_table, decide_order, take_order, make_food, take_food, serve_food, eat, move])

    problem.add_goal(Holds(server_1))

    # MaximizeExpressionOnFinalState(Revenue)

    # how to add timer?
    # print(total_distances)

    return problem


# Check for planners
from unified_planning.shortcuts import get_environment

env = get_environment()
# for engine_name in env.factory.engines:
#     engine = meta_engine(engine_name)
#     print(f"Engine: {engine_name}")
#     print(f"  Supports OBJECT_FLUENTS: {engine.supports('OBJECT_FLUENTS')}")
#     print(f"  Supports FLAT_TYPING: {engine.supports('FLAT_TYPING')}")
#     print(f"  Supports ACTION_BASED: {engine.supports('ACTION_BASED')}")

print(env.factory.engines)


# Create and solve the problem
problem = create_diner_problem()
print(problem)
with OneshotPlanner(problem_kind=problem.kind, name="tamer") as planner:
    result = planner.solve(problem)
    plan = result.plan
    if plan is not None:
        print(f"{planner.name} returned:")
        for start, action, duration in plan.timed_actions:
            if duration != None:
                print(f"{float(start)}: {action} [{float(duration)}]")
            else:
                print(f"{float(start)}: {action}")
    else:
        print("No plan found.")

print("Done")