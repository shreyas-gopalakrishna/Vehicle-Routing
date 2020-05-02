class CVRP2_GUROBI:
		def __init__(self, numberOfCustomers, numberOfVehicles, capacityOfVehicle, demandOfCustomers, costMatrix, A, L, T):
				self.numberOfCustomers = numberOfCustomers
				self.numberOfVehicles  = numberOfVehicles
				self.capacityOfVehicle = capacityOfVehicle
				self.demandOfCustomers = demandOfCustomers
				self.costMatrix        = costMatrix
				self.A = A
				self.L = L
				self.T = T
				self.N = len(self.costMatrix)
				self.initializeLP()

		def initializeLP(self):
				self.cvrpLP = gp.Model("CVRP_2")
				objective = None
				x, y = [], []
				u, v = [], []

				# Decision Variables
				for i in range(self.N):
					xRow = []
					y.append(self.cvrpLP.addVar(name='y('+str(i)+')', vtype=GRB.CONTINUOUS, lb=0))
					u.append(self.cvrpLP.addVar(name='u('+str(i)+')', vtype=GRB.CONTINUOUS, lb=0))
					v.append(self.cvrpLP.addVar(name='v('+str(i)+')', vtype=GRB.CONTINUOUS, lb=0))
					for j in range(self.N):
						xRow.append(self.cvrpLP.addVar(name='x('+str(i)+','+str(j)+')', vtype=GRB.BINARY, lb=0, ub=1))
					x.append(xRow)

				# Adding objective
				for tup in self.A:
					objective += costMatrix[tup[0]][tup[1]] * x[tup[0]][tup[1]]
				self.cvrpLP.setObjective(objective,GRB.MINIMIZE)

				# Adding constraint 1
				for j in range(1, self.N - 1):
					constraint1 = None
					for i in range(1, self.N):
						if (i != j):
							if(constraint1 == None):
								constraint1 = x[i][j]
							else:
								constraint1 = constraint1 + x[i][j]
					self.cvrpLP.addConstr(constraint1 == 1)

				# Adding constraint 2
				for i in range(1, self.N - 1):
					constraint2 = None
					for j in range(1, self.N - 1):
						if (i != j):
							if(constraint2 == None):
								constraint2 = x[i][j]
							else:
								constraint2 = constraint2 + x[i][j]
					constraint2 = constraint2 + x[i][self.N - 1]
					self.cvrpLP.addConstr(constraint2 == 1)

				# Adding constraint 3
				constraint3 = None
				for i in range(1, self.N - 1):
					if(constraint3 == None):
						constraint3 = x[0][i]
					else:
						constraint3 = constraint3 + x[0][i]
				self.cvrpLP.addConstr(constraint3 == self.numberOfVehicles)

				# Adding constraint 4
				constraint4 = None
				for i in range(1, self.N - 1):
					if(constraint4 == None):
						constraint4 = x[i][self.N - 1]
					else:
						constraint4 = constraint4 + x[i][self.N - 1]
				self.cvrpLP.addConstr(constraint4 == self.numberOfVehicles)

				# Adding constraint 5
				for tup in self.A:
					self.cvrpLP.addConstr(y[tup[0]] - y[tup[1]] + (self.L + 1) * x[tup[0]][tup[1]] <= self.L)

				# Adding constraint 6
				self.cvrpLP.addConstr(y[-1] - y[0] <= self.L + 1)

				# Adding constraint 7
				for i, j in self.A:
					self.cvrpLP.addConstr(u[i] - u[j] + self.capacityOfVehicle * x[i][j] <= self.capacityOfVehicle - self.demandOfCustomers[j])

				# Adding constraint 8
				self.cvrpLP.addConstr( u[-1] - u[0] == self.capacityOfVehicle)

				# Adding constraint 9
				for i, j in self.A:
					self.cvrpLP.addConstr(v[i] - v[j] + (self.costMatrix[i][j] + self.T) * x[i][j] <= self.T)
				
				# Adding constrint 10
				self.cvrpLP.addConstr(v[-1] - v[0] == self.T)

				# Adding flow constraint
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

		def solve(self):
				status = self.cvrpLP.optimize()
				print(status)
		
		def getResult(self):
				print("Objective value: ", self.cvrpLP.ObjVal)
				for v in self.cvrpLP.getVars():
						print(v.varName, " = ", v.x)
				return self.cvrpLP