# PuLP class for vehicle routing

class CVRP:
	def __init__(self, numberOfCustomers, numberOfVehicles, capacityOfVehicle, demandOfCustomers, costMatrix):
		self.numberOfCustomers = numberOfCustomers
		self.numberOfVehicles  = numberOfVehicles
		self.capacityOfVehicle = capacityOfVehicle
		self.demandOfCustomers = demandOfCustomers
		self.costMatrix        = costMatrix
		self.initializeLP()
	
	def initializeLP(self):
		self.cvrpLP = gp.Model("CVRP")
		objective = None
		x,y = [], []
		constraint1 = None
		
		# objective function and variables
		for i in range(len(costMatrix)): #adding depot
			xTemp1 = []
			for j in range(len(costMatrix)):
				if(i != j):
					xTemp2 = self.cvrpLP.addVar(name='x('+str(i)+','+str(j)+')', vtype=GRB.BINARY, lb=0, ub=1)
					xTemp1.append(xTemp2)
					objective += xTemp2 * costMatrix[i][j]
				else:
					xTemp1.append(None)
			x.append(xTemp1)
		self.cvrpLP.setObjective(objective, GRB.MINIMIZE) 
		
		for i in range(len(costMatrix)):
			y.append(self.cvrpLP.addVar(name='y'+str(i), , vtype=GRB.CONTINUOUS, lb=0))
		
		
		# constraints
		# ensure that all customers are visited exactly once
		for i in range(1, len(costMatrix)-1): #adding depot
			constraint1 = None
			for j in range(1, len(costMatrix)):
				if(i != j):
					if(constraint1 == None):
						constraint1 = x[i][j]
					else:
						constraint1 = constraint1 + x[i][j]
			self.cvrpLP.addConstr(constraint1 == 1)
		
		# limits the maximum number of routes to the number of vehicles
		constraint2 = None
		for j in range(1, len(costMatrix)-1): #not include depot
			if(constraint2 == None):
				constraint2 = x[0][j]
			else:
				constraint2 = constraint2 + x[0][j]
		self.cvrpLP.addConstr(constraint2 <= self.numberOfVehicles)

		# limits the maximum number of routes to the number of vehicles
		# constraint2 = None
		# for j in range(1, len(costMatrix) - 1): #not include depot
		#     constraint2 += x[j][-1]
		# self.cvrpLP += constraint2 <= self.numberOfVehicles
		
		# ensure together that the vehicle capacity is not exceeded
		for i in range(len(costMatrix)):
			constarint3a, constarint3b  = None, None
			constarint3a = self.demandOfCustomers[i] <= y[i] 
			constarint3b = y[i] <= self.capacityOfVehicle
			self.cvrpLP.addConstr(constarint3a)
			self.cvrpLP.addConstr(constarint3b)
		
		# ensure together that the vehicle capacity is not exceeded
		for i in range(len(costMatrix)): #adding depot
			for j in range(len(costMatrix)):
				constraint4 = None
				if(i != j):
					constraint4 = y[j] >= y[i] + self.demandOfCustomers[j]*x[i][j] - self.capacityOfVehicle*(1-x[i][j])
					self.cvrpLP.addConstr(constraint4)
		
		#guarantee the correct flow of vehicles through the arcs, by stating that if a vehicle arrives to a node
		#then it must depart from this node
		for h in range(1, len(costMatrix)-1):
			constraint5a, constraint5b = None, None
			for i in range(0, len(costMatrix)-1):
				if(i != h):
					if(constraint5a == None):
						constraint5a = x[i][h]
					else:
						constraint5a = constraint5a + x[i][h]
			for j in range(1, len(costMatrix)):
				if(j != h):
					if(constraint5b == None):
						constraint5b = x[h][j]
					else:
						constraint5b = constraint5b + x[h][j]
			self.cvrpLP.addConstr(constraint5a - constraint5b == 0)
		
		# print(self.cvrpLP)
		
	def solve(self):
		status = self.cvrpLP.optimize()
		print(status)
	
	def getResult(self):
		print("Objective value: ", self.cvrpLP.ObjVal)
		for v in self.cvrpLP.getVars():
			print(v.varName, " = ", v.x)
