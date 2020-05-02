class VRPSDP_GUROBI:
	def __init__(self, costMatrix, demand, pickup, numberOfVehicles, capacityOfVehicle):
		self.costMatrix = costMatrix
		self.n = len(costMatrix)
		self.demand = demand
		self.pickup = pickup
		self.numberOfVehicles = numberOfVehicles
		self.capacityOfVehicle = capacityOfVehicle
		self.initialzeLP()

	def initialzeLP(self):
		self.lp = gp.Model('VRP-SDP')
		x, R, P = [], [], []

		# Create decision variables
		for i in range(self.n):
			xRow, RRow, PRow = [], [], []
			for j in range(self.n):
				xRow.append(self.cvrpLP.addVar(name='x('+str(i)+","+str(j)+")", vtype=GRB.BINARY, lb=0, ub=1))
				RRow.append(self.cvrpLP.addVar(name='R('+str(i)+","+str(j)+")", vtype=GRB.CONTINUOUS, lb=0))
				PRow.append(self.cvrpLP.addVar(name='P('+str(i)+","+str(j)+")", vtype=GRB.CONTINUOUS, lb=0))
			x.append(xRow)
			R.append(RRow)
			P.append(PRow)

		# Create objective
		objective = None
		for i in range(self.n):
			for j in range(self.n):
				objective += self.costMatrix[i][j] * x[i][j]
		self.cvrpLP.setObjective(objective,GRB.MINIMIZE)

		# constraint 1
		for j in range(1, self.n):
			const1 = None
			for i in range(self.n):
				if(const1 == None):
					const1 = x[i][j]
				else:
					const1 = const1 + x[i][j]
			self.cvrpLP.addConstr(const1 == 1)
		
		# constraint 2
		for j in range(1, self.n):
			const2 = None
			for i in range(self.n):
				if(const2 == None):
					const2 = x[j][i]
				else:
					const2 = const2 + x[j][i]
			self.cvrpLP.addConstr(const2 == 1)

		# constraint 3
		for j in range(1, self.n):
			const3a, const3b = None, None
			for i in range(self.n):
				if(const3a == None):
					const3a = R[i][j]
				else:
					const3a = const3a + R[i][j]
				if(const3b == None):
					const3b = R[j][i]
				else:
					const3b = const3b + R[j][i]
			self.cvrpLP.addConstr(const3a - self.demand[j] == const3b)

		# constraint 4
		for j in range(1, self.n):
			const4a, const4b = None, None
			for i in range(self.n):
				if(const4a == None):
					const4a = P[i][j]
				else:
					const4a = const4a + P[i][j]
				if(const4b == None):
					const4b = P[j][i]
				else:
					const4b = const4b + P[j][i]
			self.cvrpLP.addConstr(const4a + self.pickup[j] == const4b)

		# constraint 5
		const5 = None
		for i in range(1, self.n):
			if(const5 == None):
				const5 = P[0][i]
			else:
				const5 = const5 + P[0][i]
		self.cvrpLP.addConstr(const5 == 0)

		# constraint 6
		const6 = None
		for i in range(1, self.n):
			if(const6 == None):
				const6 = R[i][0]
			else:
				const6 = const6 + R[i][0]
		self.cvrpLP.addConstr(const6 == 0)

		# constraint 7
		for i in range(self.n):
			for j in range(self.n):
				self.cvrpLP.addConstr(R[i][j] + P[i][j] <= self.capacityOfVehicle * x[i][j])

		# constraint 8
		for i in range(1, self.n):
			self.cvrpLP.addConstr(x[0][i] <= self.numberOfVehicles)

	def solve(self):
		status = self.cvrpLP.optimize()
		print(status)
	
	def getResult(self):
		print("Objective value: ", self.cvrpLP.ObjVal)
		for v in self.cvrpLP.getVars():
			print(v.varName, " = ", v.x)
		return self.cvrpLP